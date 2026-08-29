import copy

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase
from opendbc.car.toyota.tss3 import decode_eps_394_state_candidates
from opendbc.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, EPS_SCALE

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

    self.pcm_follow_distance = 0

    self.acc_type = 1
    self.lkas_hud = {}
    self.gvc = 0.0
    self.secoc_synchronization = None
    # Read-only TSS3 policy inputs. These remain outside the public CarState
    # fault/engagement contract until their live transitions are validated.
    self.tss3_ready_status = False
    self.tss3_steering_fault_inhibit_status = False
    self.tss3_driver_torque_invalid = False
    self.tss3_alt_telemetry_seen = False
    self.tss3_motor_current_alt_raw = 0
    self.tss3_alt_steering_torque = 0.0
    self.tss3_status_351_seen = False
    self.tss3_status_351_code = 0
    self.tss3_status_351_flag = False
    self.tss3_fault_394_seen = False
    self.tss3_fault_394_projection = (0, 0, 0, 0)
    self.tss3_fault_394_state_candidates: tuple[int, ...] = ()
    self.tss3_fault_394_state: int | None = None
    self.tss3_lateral_request_seen = False
    self.tss3_target_lateral_id = 0
    self.tss3_lateral_request_angle = 0.0
    self.tss3_lateral_request_sequence = 0

  @staticmethod
  def _tss3_message_seen(cp: CANParser, message: str) -> bool:
    return any(int(ts) != 0 for ts in cp.ts_nanos[message].values())

  def _update_tss3(self, cp: CANParser) -> structs.CarState:
    # TSS3 Corolla support remains intentionally read-only. Promote only fields
    # with target-native firmware + dynamic evidence; do not borrow TSS2 fault,
    # override-threshold, cruise, or readiness semantics.
    ret = structs.CarState()

    self.secoc_synchronization = copy.copy(cp.vl["SECOC_SYNCHRONIZATION"])

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
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

    can_gear = int(cp.vl["GEAR_PACKET_HYBRID"]["GEAR"])
    if self.CP.carFingerprint == CAR.TOYOTA_CAMRY_TSS3:
      # Exact same-car captures close P=0/R=1/N=2/D=3/B=4 with valid Toyota checksums.
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    else:
      # The retained Corolla route only exercises raw 3=D; do not transfer the
      # Camry selector enum across platforms merely because the DBC carrier is shared.
      ret.gearShifter = structs.CarState.GearShifter.drive if can_gear == 3 else structs.CarState.GearShifter.unknown

    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    # 0x176's wire shape/checksum are retained on both observed TSS3 Corolla
    # captures, but neither exercises an active-cruise transition. Keep cruise
    # neutral in CarState until that semantic transition is observed; the raw
    # prior-art fields remain available in the DBC for inspection.
    ret.cruiseState.enabled = False
    ret.cruiseState.available = False

    # Exact H/F firmware + Techstream close physical driver torque on 0x030:
    # signed B8 * 0.1 Nm + signed B17[3:0] * 0.01 Nm. The DBC applies those
    # component scales, so addition reconstructs the native torque intermediate.
    # If the target-native validity gate asserts, suppress the value rather than
    # exposing an invalid torque sample.
    driver_torque_valid = cp.vl["TSS3_EPS_TELEMETRY"]["DRIVER_TORQUE_INVALID"] == 0
    ret.steeringTorque = (cp.vl["TSS3_EPS_TELEMETRY"]["STEERING_WHEEL_TORQUE_COARSE"] +
                          cp.vl["TSS3_EPS_TELEMETRY"]["STEERING_WHEEL_TORQUE_FINE"]) if driver_torque_valid else 0.0
    ret.steeringTorqueEps = 0.0
    self.tss3_ready_status = bool(cp.vl["TSS3_READY_STATUS"]["READY_STATUS"])
    self.tss3_driver_torque_invalid = not driver_torque_valid
    self.tss3_steering_fault_inhibit_status = bool(cp.vl["TSS3_EPS_TELEMETRY"]["STEERING_FAULT_INHIBIT_STATUS"])
    # Camry 0x08A is a secured-looking lateral request representation, not
    # exact-F33 normal ingress or generated-COM transmit. Expose its recovered
    # fields read-only. Its producer/SecOC ownership is unresolved, and stock
    # LTA does not establish or require an 0x08A-to-B6 transform.
    self.tss3_lateral_request_seen = self._tss3_message_seen(cp, "TSS3_LATERAL_REQUEST")
    self.tss3_target_lateral_id = int(cp.vl["TSS3_LATERAL_REQUEST"]["TARGET_LATERAL_ID"])
    self.tss3_lateral_request_angle = cp.vl["TSS3_LATERAL_REQUEST"]["LATERAL_REQUEST_ANGLE"]
    self.tss3_lateral_request_sequence = int(cp.vl["TSS3_LATERAL_REQUEST"]["SEQUENCE"])

    # 0x4A3/0x351/0x394 are retained by the exact F33 Tx table. Their static
    # wire projections are useful policy inputs, but the current normal-harness
    # Camry captures do not observe their route. Track both value and presence so
    # an absent message cannot be confused with a valid all-zero/normal value.
    self.tss3_alt_telemetry_seen = self._tss3_message_seen(cp, "TSS3_ALT_STEERING_TELEMETRY")
    self.tss3_motor_current_alt_raw = int(cp.vl["TSS3_ALT_STEERING_TELEMETRY"]["MOTOR_CURRENT_ALT_RAW"])
    self.tss3_alt_steering_torque = cp.vl["TSS3_ALT_STEERING_TELEMETRY"]["STEERING_WHEEL_TORQUE"]
    self.tss3_status_351_seen = self._tss3_message_seen(cp, "TSS3_EPS_STATUS_351")
    self.tss3_status_351_code = int(cp.vl["TSS3_EPS_STATUS_351"]["STATUS_CODE"])
    self.tss3_status_351_flag = bool(cp.vl["TSS3_EPS_STATUS_351"]["STATUS_FLAG"])
    self.tss3_fault_394_seen = self._tss3_message_seen(cp, "TSS3_EPS_FAULT_STATUS_394")
    self.tss3_fault_394_projection = (
      int(cp.vl["TSS3_EPS_FAULT_STATUS_394"]["STATUS_TABLE_COLUMN_4"]),
      int(cp.vl["TSS3_EPS_FAULT_STATUS_394"]["STATUS_TABLE_COLUMN_1"]),
      int(cp.vl["TSS3_EPS_FAULT_STATUS_394"]["STATUS_TABLE_COLUMN_2"]),
      int(cp.vl["TSS3_EPS_FAULT_STATUS_394"]["STATUS_TABLE_COLUMN_3"]),
    )
    self.tss3_fault_394_state_candidates = decode_eps_394_state_candidates(self.tss3_fault_394_projection) if self.tss3_fault_394_seen else ()
    self.tss3_fault_394_state = self.tss3_fault_394_state_candidates[0] if len(self.tss3_fault_394_state_candidates) == 1 else None

    # The legacy Toyota STEER_THRESHOLD is in the old 0x260 raw domain. No H/F
    # physical driver-override threshold has been validated yet, so do not reuse
    # it for the N.m quantity above. Likewise, STEERING_FAULT_INHIBIT_STATUS is a
    # proved selected steering fault/inhibit aggregate, not an exhaustive EPS-fault
    # state, and there is no safe mapping to openpilot's
    # temporary/permanent fault split or to DID 0x1033 Ready Status yet.
    ret.steeringPressed = False
    ret.steerFaultTemporary = False
    ret.steerFaultPermanent = False

    return ret

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    if self.CP.flags & ToyotaFlags.TSS3:
      return self._update_tss3(cp)

    cp_cam = can_parsers[Bus.cam]

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
      # Required signals are present in both tracked driving captures. Gear,
      # cruise and low-rate body state are useful but specimen/trim dependent,
      # so they are ignored for alive checking.
      pt_messages = [
        ("SECOC_SYNCHRONIZATION", 10),
        ("STEER_ANGLE_SENSOR", 100),
        ("TSS3_EPS_TELEMETRY", 100),
        ("WHEEL_SPEEDS", 100),
        ("BRAKE_MODULE", 50),
        ("GAS_PEDAL", 40),
        ("GEAR_PACKET_HYBRID", float('nan')),
        ("PCM_CRUISE", float('nan')),
        # Exact H PDU29 signal154: 0x51E B0[7] -> DID 0x1033 Ready Status.
        # Parse for read-only observation; policy use remains gated on a Ready transition.
        ("TSS3_READY_STATUS", float('nan')),
        # Upstream lateral request; passive only and not alive-critical.
        ("TSS3_LATERAL_REQUEST", float('nan')),
        # Exact F33 static Tx carriers; no alive check until relay-correct live observation.
        ("TSS3_ALT_STEERING_TELEMETRY", float('nan')),
        ("TSS3_EPS_STATUS_351", float('nan')),
        ("TSS3_EPS_FAULT_STATUS_394", float('nan')),
        ("BLINKERS_STATE", float('nan')),
        ("BODY_CONTROL_STATE", float('nan')),
      ]
      pt_bus = 1 if CP.flags & ToyotaFlags.TSS3_PT_BUS1 else 0
      return {Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, pt_bus)}

    pt_messages = [
      ("BLINKERS_STATE", float('nan')),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
