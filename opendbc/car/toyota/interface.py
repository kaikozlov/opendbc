from opendbc.car import Bus, structs, get_safety_config, uds
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.carcontroller import CarController
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.values import Ecu, CAR, DBC, ToyotaFlags, CarControllerParams, TSS2_CAR, RADAR_ACC_CAR, NO_DSU_CAR, \
                                                  MIN_ACC_SPEED, EPS_SCALE, NO_STOP_TIMER_CAR, ToyotaSafetyFlags, UNSUPPORTED_DSU_CAR
from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, ToyotaSafetyFlagsSP

SteerControlType = structs.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  DRIVABLE_GEARS = (structs.CarState.GearShifter.sport,)

  def __init__(self, CP, CP_SP=None):
    self._upstream_api_compat = CP_SP is None
    super().__init__(CP, CP_SP if CP_SP is not None else structs.CarParamsSP())

  def update(self, can_packets):
    ret = super().update(can_packets)
    return ret[0] if self._upstream_api_compat else ret

  def apply(self, c, c_sp=None, now_nanos=None):
    # Keep compatibility with upstream tools and retained TSS3 tests, which
    # predate CarControlSP and pass the timestamp as the second argument.
    if isinstance(c_sp, int) and now_nanos is None:
      now_nanos = c_sp
      c_sp = None
    return super().apply(c, c_sp if c_sp is not None else structs.CarControlSP(), now_nanos)

  @staticmethod
  def get_pid_accel_limits(CP, CP_SP, current_speed=None, cruise_speed=None):
    if cruise_speed is None:
      cruise_speed = current_speed
      current_speed = CP_SP
    return CarControllerParams(CP).ACCEL_MIN, CarControllerParams(CP).ACCEL_MAX

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "toyota"

    # TSS3 Corolla powertrains share one openpilot platform and the same EPS application.
    # Keep the normal HYBRID flag meaningful even though TSS3 returns before the
    # legacy Toyota detection. Normalize Cap'n Proto enum values before set
    # membership: _DynamicEnum hashes differ from their equal integer Ecu values.
    # GTS independently distinguishes Corolla HV by an added category-466 Brake
    # Booster; the retained HV route also carries the generation-native 0x127.
    found_ecus = {fw.ecu.raw for fw in car_fw}
    if candidate == CAR.TOYOTA_COROLLA_TSS3 and (found_ecus & {Ecu.hybrid, Ecu.electricBrakeBooster} or
                                                    0x127 in fingerprint.get(1, {})):
      ret.flags |= ToyotaFlags.HYBRID.value

    if ret.flags & ToyotaFlags.TSS3:
      ret.steerControlType = SteerControlType.angle
      # Camry object geometry, qualifier and lifecycle are verified against
      # retained source frames; other TSS3 platforms have no radar DBC mapping.
      ret.radarUnavailable = Bus.radar not in DBC[candidate]
      ret.openpilotLongitudinalControl = False
      ret.autoResumeSng = False
      ret.minEnableSpeed = -1.
      ret.centerToFront = ret.wheelbase * 0.44

      if candidate == CAR.TOYOTA_CAMRY_TSS3:
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.toyota)]
        ret.safetyConfigs[0].safetyParam = (EPS_SCALE[candidate] |
                                             ToyotaSafetyFlags.F33.value)
        # The physical request-plane harness is self-identifying: chassis/state
        # lives on bus0 while the FRC-native 0x08A source lives on bus2. Select
        # host 0x08A ownership from that observed topology, not a private Param.
        relay_request_plane = 0x025 in fingerprint.get(0, {}) and 0x08A in fingerprint.get(2, {})
        if relay_request_plane:
          ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.TSS3_08A_HOST.value
          ret.alphaLongitudinalAvailable = True
          ret.openpilotLongitudinalControl = alpha_long
          ret.autoResumeSng = alpha_long
          # FRC remains the native cruise engagement/set-speed owner while
          # openpilot replaces its longitudinal actuation request.
          ret.pcmCruise = True
        ret.dashcamOnly = False
        # The EPS-resident helper owns native B6 signing; openpilot owns only
        # the bounded C7 sideband and therefore needs no host SecOC key.
        ret.secOcRequired = False
        ret.minSteerSpeed = 0.
        ret.steerAtStandstill = True
        # Stock Toyota-B exposes this source on bus1; the request-plane repin
        # moves the FRC vocabulary, including BSM, to bus2.
        ret.enableBsm = 0x3F6 in fingerprint[2 if relay_request_plane else 1]
        ret.steerActuatorDelay = 0.18
        ret.steerLimitTimer = 0.8
      elif candidate == CAR.TOYOTA_COROLLA_TSS3:
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.toyota)]
        ret.safetyConfigs[0].safetyParam = (EPS_SCALE[candidate] |
                                             ToyotaSafetyFlags.TSS3_SIGNER.value |
                                             ToyotaSafetyFlags.COROLLA_HF.value)
        ret.dashcamOnly = False
        # The RAM-resident helper signs a native EPS-local B6. openpilot sends
        # only the unified functional-0x777 C7 control used across TSS3 targets.
        ret.secOcRequired = False
        ret.minSteerSpeed = 0.
        ret.steerAtStandstill = True
        # Corolla TSS3 can follow stock ACC through a stop, but the retained
        # contributor drives require the driver to establish/resume cruise below
        # Toyota's 19 mph set-speed floor. Keep the native no-entry threshold.
        ret.minEnableSpeed = MIN_ACC_SPEED
        ret.enableBsm = 0x3F6 in fingerprint[1]
        ret.steerActuatorDelay = 0.18
        ret.steerLimitTimer = 0.8
      else:
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]
        ret.dashcamOnly = True

      if not ret.dashcamOnly:
        # Stock Toyota-B has no independently suppressible 0x08A source. The
        # Camry request-plane repin does, and advertises Alpha Long only there.
        if not ret.openpilotLongitudinalControl:
          ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

      return ret

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = EPS_SCALE[candidate]

    # BRAKE_MODULE is on a different address for these cars
    if DBC[candidate][Bus.pt] == "toyota_new_mc_pt_generated":
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.ALT_BRAKE.value

    if ret.flags & ToyotaFlags.SECOC.value:
      ret.secOcRequired = True
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.SECOC.value
      ret.dashcamOnly = is_release

    if ret.flags & ToyotaFlags.ANGLE_CONTROL:
      ret.steerControlType = SteerControlType.angle
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.LTA.value

      # LTA control can be more delayed and winds up more often
      ret.steerActuatorDelay = 0.18
      ret.steerLimitTimer = 0.8
    else:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

      ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
      ret.steerLimitTimer = 0.4

    stop_and_go = bool(ret.flags & ToyotaFlags.TSS2)

    # In TSS2 cars, the camera does long control
    found_ecus = [fw.ecu for fw in car_fw]

    if Ecu.hybrid in found_ecus:
      ret.flags |= ToyotaFlags.HYBRID.value

    if candidate == CAR.TOYOTA_PRIUS:
      stop_and_go = True
      # Only give steer angle deadzone to for bad angle sensor prius
      for fw in car_fw:
        if fw.ecu == "eps" and not fw.fwVersion == b'8965B47060\x00\x00\x00\x00\x00\x00':
          ret.steerActuatorDelay = 0.25
          CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.2)

    elif candidate in (CAR.LEXUS_RX, CAR.LEXUS_RX_TSS2):
      stop_and_go = True
      ret.wheelSpeedFactor = 1.035

    elif candidate in (CAR.TOYOTA_AVALON, CAR.TOYOTA_AVALON_2019, CAR.TOYOTA_AVALON_TSS2):
      # starting from 2019, all Avalon variants have stop and go
      # https://engage.toyota.com/static/images/toyota_safety_sense/TSS_Applicability_Chart.pdf
      stop_and_go = candidate != CAR.TOYOTA_AVALON

    elif candidate in (CAR.TOYOTA_CHR, CAR.TOYOTA_CAMRY, CAR.TOYOTA_SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_LS, CAR.LEXUS_NX):
      # TODO: Some of these platforms are not advertised to have full range ACC, do they really all have sng?
      stop_and_go = True

    ret.centerToFront = ret.wheelbase * 0.44

    # TODO: Some TSS-P platforms have BSM, but are flipped based on region or driving direction.
    # Detect flipped signals and enable for C-HR and others
    ret.enableBsm = 0x3F6 in fingerprint[0] and bool(ret.flags & ToyotaFlags.TSS2)

    # No radar dbc for cars without DSU which are not TSS 2.0
    # TODO: make an adas dbc file for dsu-less models
    ret.radarUnavailable = Bus.radar not in DBC[candidate] or candidate in (NO_DSU_CAR - TSS2_CAR)

    # since we don't yet parse radar on TSS2/TSS-P radar-based ACC cars, gate longitudinal behind experimental toggle
    if ret.flags & ToyotaFlags.RADAR_ACC:
      ret.alphaLongitudinalAvailable = True

      # Disabling radar is only supported on TSS2 radar-ACC cars
      if alpha_long and candidate in RADAR_ACC_CAR:
        ret.flags |= ToyotaFlags.DISABLE_RADAR.value

    # openpilot longitudinal enabled by default:
    #  - cars w/ DSU disconnected
    #  - TSS2 cars with camera sending ACC_CONTROL where we can block it
    # openpilot longitudinal behind experimental long toggle:
    #  - TSS2 radar ACC cars (disables radar)

    ret.openpilotLongitudinalControl = ((bool(ret.flags & ToyotaFlags.TSS2) and not (ret.flags & ToyotaFlags.RADAR_ACC)) or
                                        bool(ret.flags & ToyotaFlags.DISABLE_RADAR.value))

    ret.autoResumeSng = ret.openpilotLongitudinalControl and candidate in NO_STOP_TIMER_CAR

    if not ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1. if stop_and_go else MIN_ACC_SPEED

    if ret.flags & ToyotaFlags.TSS2:
      ret.flags |= ToyotaFlags.RAISED_ACCEL_LIMIT.value

      # Hybrids have much quicker longitudinal actuator response
      if ret.flags & ToyotaFlags.HYBRID.value:
        ret.longitudinalActuatorDelay = 0.05

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    if candidate in UNSUPPORTED_DSU_CAR:
      ret.safetyParam |= ToyotaSafetyFlagsSP.UNSUPPORTED_DSU

    # Detect smartDSU, which intercepts ACC_CMD from the DSU (or radar) allowing openpilot to send it
    # 0x2AA is sent by a similar device which intercepts the radar instead of DSU on NO_DSU_CARs
    if 0x2FF in fingerprint[0] or (0x2AA in fingerprint[0] and candidate in NO_DSU_CAR):
      ret.flags |= ToyotaFlagsSP.SMART_DSU.value

    if 0x2AA in fingerprint[0] and candidate in NO_DSU_CAR:
      ret.flags |= ToyotaFlagsSP.RADAR_CAN_FILTER.value

    # Detect ZSS, which allows sunnypilot to utilize an improved angle sensor for some Toyota vehicles
    # https://github.com/zorrobyte/betterToyotaAngleSensorForOP
    if 0x23 in fingerprint[0] and not stock_cp.flags & ToyotaFlags.SECOC:
      ret.flags |= ToyotaFlagsSP.ZSS.value

    if candidate == CAR.TOYOTA_PRIUS:
      if ret.flags & ToyotaFlagsSP.ZSS:
        stock_cp.steerRatio = 15.0
        stock_cp.mass = 3370.

        # reuse logic from _get_params
        # Only give steer angle deadzone to for bad angle sensor prius
        for fw in car_fw:
          if fw.ecu == "eps" and not fw.fwVersion == b'8965B47060\x00\x00\x00\x00\x00\x00':
            stock_cp.steerActuatorDelay = 0.25
            CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning, steering_angle_deadzone_deg=0.0)

    use_sdsu = bool(ret.flags & ToyotaFlagsSP.SMART_DSU)

    stock_cp.minEnableSpeed = -1. if use_sdsu else stock_cp.minEnableSpeed

    # reuse logic from _get_params
    # if the smartDSU is detected, openpilot can send ACC_CONTROL and the smartDSU will block it from the DSU or radar.
    # since we don't yet parse radar on TSS2/TSS-P radar-based ACC cars, gate longitudinal behind experimental toggle
    stock_cp.alphaLongitudinalAvailable = use_sdsu or candidate in RADAR_ACC_CAR

    if use_sdsu:
      use_sdsu = use_sdsu and alpha_long
      stock_cp.flags &= ~ToyotaFlags.DISABLE_RADAR.value
    elif candidate in (RADAR_ACC_CAR | NO_DSU_CAR):
      # Disabling radar is only supported on TSS2 radar-ACC cars
      if alpha_long and candidate in RADAR_ACC_CAR:
        stock_cp.flags |= ToyotaFlags.DISABLE_RADAR.value

    # openpilot longitudinal enabled by default:
    #  - TSS2 cars with camera sending ACC_CONTROL where we can block it
    # openpilot longitudinal behind experimental long toggle:
    #  - cars w/ smartDSU or CAN filter installed
    #  - TSS2 radar ACC cars w/o smartDSU installed (disables radar)
    stock_cp.openpilotLongitudinalControl = use_sdsu or \
      candidate in (TSS2_CAR - RADAR_ACC_CAR) or \
      bool(stock_cp.flags & ToyotaFlags.DISABLE_RADAR)

    ret.enableGasInterceptor = 0x201 in fingerprint[0] and stock_cp.openpilotLongitudinalControl and \
                               not stock_cp.flags & ToyotaFlags.SECOC

    if ret.enableGasInterceptor:
      ret.safetyParam |= ToyotaSafetyFlagsSP.GAS_INTERCEPTOR
      stock_cp.minEnableSpeed = -1.

    if not stock_cp.openpilotLongitudinalControl:
      stock_cp.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value
    else:
      stock_cp.safetyConfigs[0].safetyParam &= ~ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    return ret

  @staticmethod
  def init(CP, CP_SP, can_recv, can_send, communication_control=None):
    # disable radar if alpha longitudinal toggled on radar-ACC car without CAN filter/smartDSU
    if CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if communication_control is None:
        communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])
      disable_ecu(can_recv, can_send, bus=0, addr=0x750, sub_addr=0xf, com_cont_req=communication_control)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    # re-enable radar if alpha longitudinal toggled on radar-ACC car
    communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX, uds.MESSAGE_TYPE.NORMAL])
    CarInterface.init(CP, can_recv, can_send, communication_control)
