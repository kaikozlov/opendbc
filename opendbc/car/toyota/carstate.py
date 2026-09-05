import copy

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase
from opendbc.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, EPS_SCALE, TSS3_STEER_DRIVER_TORQUE_THRESHOLD

ButtonType = structs.CarState.ButtonEvent.Type
SteerControlType = structs.CarParams.SteerControlType

# These steering fault definitions seem to be common across LKA (torque) and LTA (angle):
# - high steer rate fault: goes to 21 or 25 for 1 frame, then 9 for 2 seconds
# - lka/lta msg drop out: goes to 9 then 11 for a combined total of 2 seconds, then 3.
#     if using the other control command, goes directly to 3 after 1.5 seconds
# - initializing: LTA can report 0 as long as STEER_TORQUE_SENSOR->STEER_ANGLE_INITIALIZING is 1,
#     and is a catch-all for LKA
TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
# - lka/lta msg drop out: 3 (recoverable)
# - prolonged high driver torque: 17 (permanent)
PERM_STEER_FAULTS = (3, 17)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    if CP.flags & ToyotaFlags.SECOC.value:
      self.shifter_values = can_define.dv["GEAR_PACKET_HYBRID"]["GEAR"]
    else:
      self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.lkas_button = 0
    self.distance_button = 0
    self.tss3_cruise_button = 0

    self.pcm_follow_distance = 0

    self.acc_type = 1
    self.lkas_hud = {}
    self.gvc = 0.0
    self.secoc_synchronization = None
    self.tss3_brake_module = None
    self.tss3_lateral_request = None

  def _update_tss3(self, cp: CANParser, cp_cam: CANParser) -> structs.CarState:
    ret = structs.CarState()

    self.secoc_synchronization = copy.copy(cp.vl["SECOC_SYNCHRONIZATION"])
    self.tss3_brake_module = copy.copy(cp.vl["BRAKE_MODULE"])

    ret.brakePressed = self.tss3_brake_module["BRAKE_PRESSED"] != 0
    ret.gasPressed = cp.vl["GAS_PEDAL"]["GAS_PEDAL_USER"] > 0

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoCluster = ret.vEgo
    ret.standstill = abs(ret.vEgoRaw) < 1e-3
    ret.vehicleSensorsInvalid = any(cp.vl["WHEEL_SPEEDS"][f"WHEEL_SPEED_{whl}_FAULT"]
                                    for whl in ("FL", "FR", "RL", "RR"))

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
    ret.carNotReady = cp.vl["TSS3_READY_STATUS"]["READY_STATUS"] == 0

    can_gear = int(cp.vl["GEAR_PACKET_HYBRID"]["GEAR"])
    if self.CP.carFingerprint == CAR.TOYOTA_CAMRY_TSS3:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    else:
      ret.gearShifter = structs.CarState.GearShifter.drive if can_gear == 3 else structs.CarState.GearShifter.unknown

    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    if self.CP.carFingerprint == CAR.TOYOTA_CAMRY_TSS3:
      switch = cp.vl["TSS3_CRUISE_SWITCH"]
      prev_cruise_button = self.tss3_cruise_button
      if switch["CANCEL_BUTTON"] and not switch["CANCEL_BUTTON_MIRROR_N"]:
        self.tss3_cruise_button = 1
      elif switch["SET_BUTTON"] and not switch["SET_BUTTON_MIRROR_N"]:
        self.tss3_cruise_button = 2
      elif switch["RES_BUTTON"] and not switch["RES_BUTTON_MIRROR_N"]:
        self.tss3_cruise_button = 3
      elif switch["MAIN_BUTTON"]:
        self.tss3_cruise_button = 4
      else:
        self.tss3_cruise_button = 0
      ret.buttonEvents = create_button_events(self.tss3_cruise_button, prev_cruise_button, {
        1: ButtonType.cancel,
        2: ButtonType.decelCruise,
        3: ButtonType.accelCruise,
        4: ButtonType.mainCruise,
      })

    ret.doorOpen = any(cp.vl["BODY_CONTROL_STATE"][door] for door in
                       ("DOOR_OPEN_FL", "DOOR_OPEN_FR", "DOOR_OPEN_RL", "DOOR_OPEN_RR"))
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0
    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])

    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cp.vl["BSM"]["L_ADJACENT"] or cp.vl["BSM"]["L_APPROACHING"])
      ret.rightBlindspot = bool(cp.vl["BSM"]["R_ADJACENT"] or cp.vl["BSM"]["R_APPROACHING"])

    driver_torque_invalid = cp.vl["TSS3_EPS_TELEMETRY"]["DRIVER_TORQUE_INVALID"] != 0
    ret.steeringTorque = (cp.vl["TSS3_EPS_TELEMETRY"]["STEERING_WHEEL_TORQUE_COARSE"] +
                          cp.vl["TSS3_EPS_TELEMETRY"]["STEERING_WHEEL_TORQUE_FINE"]) if not driver_torque_invalid else 0.0
    ret.steeringTorqueEps = 0.0
    # Normal openpilot driver-state semantics: physical torque above the
    # exact-F33 provisional threshold marks driver intervention (feeds
    # DesireHelper lane-change entry and the saturation-timer override). The
    # 0x030 torque sign convention is not yet dynamically confirmed.
    if self.CP.carFingerprint == CAR.TOYOTA_CAMRY_TSS3:
      ret.steeringPressed = abs(ret.steeringTorque) > TSS3_STEER_DRIVER_TORQUE_THRESHOLD
    # The target exposes independent torque-invalid and steering-fault/inhibit
    # status bits, but no same-car asserted/recovery fault transition yet. Keep
    # openpilot fault classification neutral rather than inventing a mapping.
    ret.steerFaultTemporary = False
    ret.steerFaultPermanent = False

    if self.CP.carFingerprint == CAR.TOYOTA_CAMRY_TSS3:
      lateral = cp_cam
      self.tss3_lateral_request = copy.copy(lateral.vl["TSS3_LATERAL_REQUEST"])
      ret.cruiseState.enabled = bool(self.tss3_lateral_request["CRUISE_OPERATING_LATCH"])
      # The retained Camry drives prove this latch follows actual MAIN activation
      # and CANCEL. No distinct TSS3 standby/main-only carrier is recovered yet.
      ret.cruiseState.available = ret.cruiseState.enabled
      set_speed_kph = float(self.tss3_lateral_request["SET_SPEED"])
      ret.cruiseState.speed = set_speed_kph * CV.KPH_TO_MS if set_speed_kph > 0 else 0.0
      cluster_set_speed = float(lateral.vl["TSS3_CRUISE_DISPLAY"]["UI_SET_SPEED"])
      is_metric = cp.vl["BODY_CONTROL_STATE_2"]["UNITS"] in (1, 2)
      if ret.cruiseState.speed != 0 and cluster_set_speed > 0:
        conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
        ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor
    else:
      ret.cruiseState.available = False
      ret.cruiseState.enabled = False

    return ret

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers.get(Bus.cam, cp)
    if self.CP.flags & ToyotaFlags.TSS3:
      return self._update_tss3(cp, cp_cam)

    ret = structs.CarState()
    cp_acc = cp_cam if (self.CP.flags & ToyotaFlags.TSS2) and not (self.CP.flags & ToyotaFlags.RADAR_ACC) else cp

    if not self.CP.flags & ToyotaFlags.SECOC.value:
      self.gvc = cp.vl["VSC1S07"]["GVC"]

    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1

    if self.CP.flags & ToyotaFlags.SECOC.value:
      self.secoc_synchronization = copy.copy(cp.vl["SECOC_SYNCHRONIZATION"])
      ret.gasPressed = cp.vl["GAS_PEDAL"]["GAS_PEDAL_USER"] > 0
      can_gear = int(cp.vl["GEAR_PACKET_HYBRID"]["GEAR"])
    else:
      ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0
      can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
      if not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
        ret.stockAeb = bool(cp_acc.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_acc.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars

    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    ret.vehicleSensorsInvalid = any(cp.vl["WHEEL_SPEEDS"][f"WHEEL_SPEED_{whl}_FAULT"]
                                    for whl in ("FL", "FR", "RL", "RR"))

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]

    # On some cars, the angle measurement is non-zero while initializing
    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      # Offset seems to be invalid for large steering angles and high angle rates
      if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # Check EPS LKA/LTA fault status
    ret.steerFaultTemporary = cp.vl["EPS_STATUS"]["LKA_STATE"] in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp.vl["EPS_STATUS"]["LKA_STATE"] in PERM_STEER_FAULTS

    if self.CP.steerControlType == SteerControlType.angle:
      ret.steerFaultTemporary = ret.steerFaultTemporary or cp.vl["EPS_STATUS"]["LTA_STATE"] in TEMP_STEER_FAULTS
      ret.steerFaultPermanent = ret.steerFaultPermanent or cp.vl["EPS_STATUS"]["LTA_STATE"] in PERM_STEER_FAULTS

      # Lane Tracing Assist control is unavailable (EPS_STATUS->LTA_STATE=0) until
      # the more accurate angle sensor signal is initialized
      if not self.accurate_steer_angle_seen:
        ret.vehicleSensorsInvalid = True

    if self.CP.flags & ToyotaFlags.UNSUPPORTED_DSU:
      # TODO: find the bit likely in DSU_CRUISE that describes an ACC fault. one may also exist in CLUTCH
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_ALT"]["UI_SET_SPEED"]
    else:
      ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
      ret.carFaultedNonCritical = cp.vl["PCM_CRUISE_SM"]["TEMP_ACC_FAULTED"] != 0
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_SM"]["UI_SET_SPEED"]

    # UI_SET_SPEED is always non-zero when main is on, hide until first enable
    is_metric = cp.vl["BODY_CONTROL_STATE_2"]["UNITS"] in (1, 2)
    if ret.cruiseState.speed != 0:
      conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
      ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor

    if self.CP.flags & ToyotaFlags.TSS2 and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      self.acc_type = cp_acc.vl["ACC_CONTROL"]["ACC_TYPE"]
      ret.stockFcw = bool(cp_acc.vl["PCS_HUD"]["FCW"])

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (not (self.CP.flags & ToyotaFlags.TSS2) and not (self.CP.flags & ToyotaFlags.UNSUPPORTED_DSU)) or \
       (self.CP.flags & ToyotaFlags.TSS2 and self.acc_type == 1):
      if self.CP.openpilotLongitudinalControl:
        ret.accFaulted = ret.accFaulted or cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    ret.cruiseState.standstill = pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = pcm_acc_status in (1, 2, 3, 4, 5, 6)

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD"])

    if not (self.CP.flags & ToyotaFlags.UNSUPPORTED_DSU):
      self.pcm_follow_distance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"]

    buttonEvents = []
    if self.CP.flags & ToyotaFlags.TSS2:
      # lkas button is wired to the camera
      prev_lkas_button = self.lkas_button
      self.lkas_button = cp_cam.vl["LKAS_HUD"]["LDA_ON_MESSAGE"]

      # Cycles between 1 and 2 when pressing the button, then rests back at 0 after ~3s
      if self.lkas_button != 0 and self.lkas_button != prev_lkas_button:
        buttonEvents.extend(create_button_events(1, 0, {1: ButtonType.lkas}) +
                            create_button_events(0, 1, {1: ButtonType.lkas}))

      if not (self.CP.flags & (ToyotaFlags.RADAR_ACC | ToyotaFlags.SECOC)):
        # distance button is wired to the ACC module (camera or radar)
        prev_distance_button = self.distance_button
        self.distance_button = cp_acc.vl["ACC_CONTROL"]["DISTANCE"]

        buttonEvents += create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    ret.buttonEvents = buttonEvents
    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.flags & ToyotaFlags.TSS3:
      pt_messages = [
        ("SECOC_SYNCHRONIZATION", 10),
        ("STEER_ANGLE_SENSOR", 100),
        ("TSS3_EPS_TELEMETRY", 100),
        ("WHEEL_SPEEDS", 100),
        ("BRAKE_MODULE", 50),
        ("GAS_PEDAL", 40),
        ("GEAR_PACKET_HYBRID", float('nan')),
        ("TSS3_READY_STATUS", float('nan')),
        ("ESP_CONTROL", float('nan')),
        ("BLINKERS_STATE", float('nan')),
        ("BODY_CONTROL_STATE", float('nan')),
        ("LIGHT_STALK", float('nan')),
      ]
      if CP.enableBsm:
        pt_messages.append(("BSM", float('nan')))
      if CP.carFingerprint == CAR.TOYOTA_CAMRY_TSS3:
        pt_messages += [
          ("TSS3_CRUISE_SWITCH", 33),
          ("BODY_CONTROL_STATE_2", float('nan')),
        ]
      else:
        pt_messages.append(("TSS3_LATERAL_REQUEST", float('nan')))

      pt_bus = 1 if CP.carFingerprint == CAR.TOYOTA_COROLLA_TSS3 else 0
      parsers = {Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, pt_bus)}
      if CP.carFingerprint == CAR.TOYOTA_CAMRY_TSS3:
        # The Toyota-B relay isolates the FRC side on Panda bus 2. Read the
        # native request there; bus 0 carries vehicle/EPS state and the replacement 0x08A output.
        parsers[Bus.cam] = CANParser(DBC[CP.carFingerprint][Bus.pt], [
          ("TSS3_LATERAL_REQUEST", 83),
          ("TSS3_CRUISE_DISPLAY", 1),
        ], 2)
      return parsers

    pt_messages = [
      ("BLINKERS_STATE", float('nan')),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
