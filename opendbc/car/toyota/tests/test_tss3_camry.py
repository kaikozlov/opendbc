import unittest

from opendbc.can import CANParser
from opendbc.car import Bus, CanData, structs
from opendbc.car.fw_versions import match_fw_to_car_exact
from opendbc.car.fw_query_definitions import PlatformResolverContext
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.toyotacan import toyota_e2e_p05_checksum
from opendbc.car.toyota.values import CAR, DBC, EPS_SCALE, CarControllerParams, ToyotaFlags, ToyotaSafetyFlags, resolve_platform
from opendbc.safety.tests.libsafety import libsafety_py


Ecu = structs.CarParams.Ecu

CAMRY_COMMON = {
  0x025: bytes.fromhex("000100005000007e0000000000000000000000000000000000000000bb6fee54"),
  # Operating zero-torque source: route 8d, segment 6, logMonoTime 3057008786467.
  0x030: bytes.fromhex("000000ffc400201b00ffc0ff9e00003f22000000ff9e007000000000b96152f6"),
  0x08A: bytes.fromhex("0000000880002d47fe462afe467fff007fffff35c000100064003c005db7797f"),
  0x0AA: bytes.fromhex("1a6f1a6f1a6f1a6f"),
  0x0FE: bytes.fromhex("567d393f0000c36200000000000000002640000000ff000000000000d54aaf10"),
  0x101: bytes.fromhex("800000010000008b"),
  0x116: bytes.fromhex("000000007b4b235a"),
  0x127: bytes.fromhex("00100000003e8d0b"),
  0x251: bytes.fromhex("c01015908030a080"),
  0x3B7: bytes.fromhex("0000000020000008"),
  0x3F6: bytes.fromhex("81ea6e0480ba4808"),
  0x51E: bytes.fromhex("80006e0000000000"),
  0x610: bytes.fromhex("00001d4ed0fffc00"),
  0x614: bytes.fromhex("00004a3000003303"),
  0x620: bytes.fromhex("000000008000001a"),
  0x622: bytes.fromhex("0000000000730000"),
}
CAMRY_LONG = bytes.fromhex("e2420d82800040034deffb000008008000bfff100a5fffd40000000000000000")
CAMRY_HUD = bytes.fromhex("140c404401ee9307")
CAMRY_RADAR = {
  0x180: bytes.fromhex("2fbd1016074a010003ff0a0e08062001ff5e0879f7eff5ff400160004000ff100901ff7005ff650291fae000ffff069df96004ff0a085405afffffff00000000"),
  0x181: bytes.fromhex("228f1016fff8000000fffffff8000000fffffff8000000fffffff8000000fffffff8000000fffffff8000000fffffff8000000fffffff8000000ffff00000000"),
  0x182: bytes.fromhex("71da1016fff8000000fffffff8000000fffffff8000000fffffff8000000fffffff8000000fffffff8000000fffffff8000000fffffff8000000ffff00000000"),
  0x183: bytes.fromhex("44fa1016080000000001420000000000010200bffcff000902018000000083000800000000010200000000048b40080001ff00094200000000040b4200000000"),
  0x184: bytes.fromhex("c3621016000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"),
  0x185: bytes.fromhex("f2511016000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"),
}


def fingerprint() -> dict[int, dict[int, int]]:
  fp = {i: {} for i in range(8)}
  fp[1] = {address: len(data) for address, data in CAMRY_COMMON.items()}
  return fp


def relay_fingerprint() -> dict[int, dict[int, int]]:
  fp = fingerprint()
  # Exact measured repin: chassis/state side on bus0, FRC source on bus2,
  # radar/object family remains on bus1. Only the topology discriminators are
  # required here; parser behavior is covered separately below.
  fp[0][0x025] = len(CAMRY_COMMON[0x025])
  fp[2][0x08A] = len(CAMRY_COMMON[0x08A])
  fp[2][0x3F6] = len(CAMRY_COMMON[0x3F6])
  return fp


def update_state(ci: CarInterface, moving: bool = False, counter_offset: int = 0, hud: bytes | None = None,
                 eps_status: int | None = None, eps_telemetry: bytes | None = None,
                 control_request: bytes | None = None, bus: int = 1, source_bus: int | None = None,
                 speed_ms: float | None = None, cruise_display: bytes | None = None, iterations: int = 20):
  state = None
  for i in range(iterations):
    frames = dict(CAMRY_COMMON)
    if eps_telemetry is not None:
      frames[0x030] = eps_telemetry
    if control_request is not None:
      frames[0x08A] = control_request
    if cruise_display is not None:
      frames[0x251] = cruise_display
    if eps_status is not None:
      eps = bytearray(frames[0x030])
      eps[6] = eps_status
      eps[7] = (sum(eps[:7]) + 0x38) & 0xFF
      frames[0x030] = bytes(eps)
    if moving:
      frames[0x0AA] = bytes.fromhex("1c001c001c001c00")
    if speed_ms is not None:
      wheel_raw = 6767 + round(speed_ms * 3.6 / 0.01)
      frames[0x0AA] = wheel_raw.to_bytes(2, "big") * 4
    source_ids = {0x08A, 0x251, 0x3F6, 0x412}
    packets = [CanData(address, data, source_bus if source_bus is not None and address in source_ids else bus)
               for address, data in frames.items()]
    if hud is not None:
      packets.append(CanData(0x412, hud, source_bus if source_bus is not None else bus))
    state = ci.update([(1_000_000_000 + (counter_offset + i) * 10_000_000, packets)])
  return state


def control(angle: float, active: bool = True, accel: float = 0.0, long_active: bool = False,
            cancel: bool = False, left_lane: bool = False, right_lane: bool = False, steer_alert: bool = False):
  cc = structs.CarControl()
  cc.enabled = True
  cc.latActive = active
  cc.longActive = long_active
  cc.cruiseControl.cancel = cancel
  cc.actuators.steeringAngleDeg = angle
  cc.actuators.accel = accel
  cc.hudControl.leftLaneVisible = left_lane
  cc.hudControl.rightLaneVisible = right_lane
  if steer_alert:
    cc.hudControl.visualAlert = structs.CarControl.HUDControl.VisualAlert.steerRequired
  return cc.as_reader()


class TestToyotaCamryTSS3(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], True, False, False)

  def test_platform_contract(self):
    self.assertTrue(self.CP.flags & ToyotaFlags.TSS3)
    self.assertTrue(self.CP.flags & ToyotaFlags.SECOC)
    self.assertFalse(self.CP.flags & ToyotaFlags.TSS2)
    self.assertFalse(self.CP.dashcamOnly)
    self.assertFalse(self.CP.secOcRequired)
    self.assertFalse(self.CP.openpilotLongitudinalControl)
    self.assertFalse(self.CP.alphaLongitudinalAvailable)
    self.assertFalse(self.CP.autoResumeSng)
    self.assertFalse(self.CP.radarUnavailable)
    self.assertEqual(DBC[CAR.TOYOTA_CAMRY_TSS3][Bus.radar], "toyota_tss3_pt_generated")
    self.assertAlmostEqual(self.CP.steerRatio, 15.3, places=3)
    # paramsd learns a multiplier of CP.tireStiffnessFront/Rear, not a
    # replacement for the factor already used to construct those values.
    self.assertAlmostEqual(self.CP.tireStiffnessFactor, 0.7933, places=4)
    self.assertAlmostEqual(self.CP.steerActuatorDelay, 0.18, places=3)
    self.assertEqual(self.CP.steerControlType, structs.CarParams.SteerControlType.angle)
    self.assertEqual(self.CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.toyota)
    self.assertTrue(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.F33)
    self.assertTrue(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL)
    self.assertEqual(DBC[CAR.TOYOTA_CAMRY_TSS3][Bus.pt], "toyota_tss3_pt_generated")
    self.assertTrue(self.CP.flags & ToyotaFlags.HAS_BSM)

  def test_relay_request_plane_is_selected_from_fingerprint_topology(self):
    stock = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], True, False, False)
    self.assertFalse(stock.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.TSS3_08A_HOST.value)

    relay = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)
    self.assertTrue(relay.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.TSS3_08A_HOST.value)
    self.assertTrue(relay.flags & ToyotaFlags.HAS_BSM)
    self.assertTrue(relay.alphaLongitudinalAvailable)
    self.assertTrue(relay.openpilotLongitudinalControl)
    self.assertTrue(relay.autoResumeSng)
    self.assertTrue(relay.pcmCruise)
    self.assertAlmostEqual(relay.longitudinalActuatorDelay, 0.2)
    self.assertEqual(list(relay.longitudinalTuning.kiBP), [0.])
    self.assertEqual(list(relay.longitudinalTuning.kiV), [0.])
    self.assertFalse(relay.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL.value)

    relay_stock_long = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], False, False, False)
    self.assertTrue(relay_stock_long.alphaLongitudinalAvailable)
    self.assertFalse(relay_stock_long.openpilotLongitudinalControl)
    self.assertTrue(relay_stock_long.pcmCruise)
    self.assertTrue(relay_stock_long.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL.value)

  def test_unqualified_camry_longitudinal_is_not_advertised(self):
    for alpha_long in (False, True):
      cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], alpha_long, False, False)
      self.assertFalse(cp.alphaLongitudinalAvailable)
      self.assertFalse(cp.openpilotLongitudinalControl)
      self.assertFalse(cp.autoResumeSng)
      self.assertTrue(cp.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL)

  def test_exact_identity_and_oem_resolver(self):
    fw = FW_VERSIONS[CAR.TOYOTA_CAMRY_TSS3]
    self.assertEqual(fw[(Ecu.eps, 0x7A1, None)], [
      bytes.fromhex("023839363546333330373030300000000038413331313333303331303000000000")])
    context = PlatformResolverContext(vin_rx_addr=0x7E8, vin_rx_bus=1)
    self.assertEqual(resolve_platform({}, "JTDAA12K0T0123456", {}, context), {str(CAR.TOYOTA_CAMRY_TSS3)})

  def test_tss3_radar_points_from_retained_object_bank(self):
    # Raw source frames exercise the default Camry radar interface.
    ri = RadarInterface(self.CP)
    packets = [CanData(address, data, 0) for address, data in CAMRY_RADAR.items()]
    rr = ri.update([(1_000_000_000, packets)])
    self.assertIsNotNone(rr)
    points = {point.trackId: point for point in rr.points}
    self.assertEqual(set(points), set(range(8)))
    self.assertAlmostEqual(points[0].dRel, 9.33, places=2)
    self.assertAlmostEqual(points[0].yRel, 0.64, places=2)
    self.assertAlmostEqual(points[2].dRel, 10.845, places=2)
    self.assertAlmostEqual(points[2].yRel, -5.2, places=2)
    self.assertAlmostEqual(points[2].vRel, -0.1, places=2)

    empty = {}
    sentinel = bytes.fromhex("fff8000000ffff") * 8
    for address in range(0x180, 0x183):
      data = bytearray(CAMRY_RADAR[address])
      data[4:60] = sentinel
      data[2] = (data[2] + 1) & 0xFF
      data[3] = (data[3] + 1) & 0xFF
      data[:2] = toyota_e2e_p05_checksum(address, data).to_bytes(2, "little")
      empty[address] = bytes(data)
    for address in range(0x183, 0x186):
      data = bytearray(CAMRY_RADAR[address])
      data[4:60] = bytes(56)
      data[2] = (data[2] + 1) & 0xFF
      data[3] = (data[3] + 1) & 0xFF
      data[:2] = toyota_e2e_p05_checksum(address, data).to_bytes(2, "little")
      empty[address] = bytes(data)
    rr = ri.update([(1_050_000_000, [CanData(address, data, 0) for address, data in empty.items()])])
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 0)

  def test_exact_identity_survives_nrtd_diagnostic_eps_miss(self):
    fw = FW_VERSIONS[CAR.TOYOTA_CAMRY_TSS3]
    eps_version = fw[(Ecu.eps, 0x7A1, None)][0]
    abs_version = fw[(Ecu.abs, 0x7B0, None)][0]

    # Normal startup: both exact control-API and corroborating chassis identities match.
    live_fw = {(0x7A1, None): {eps_version}, (0x7B0, None): {abs_version}}
    self.assertEqual(match_fw_to_car_exact(live_fw, match_brand="toyota", log=False),
                     {str(CAR.TOYOTA_CAMRY_TSS3)})

    # NRTD startup can transiently miss EPS F181. The exact F33 ABS identity is
    # enough to retain the platform instead of falling through to MOCK/dashcam mode.
    self.assertEqual(match_fw_to_car_exact({(0x7B0, None): {abs_version}}, match_brand="toyota", log=False),
                     {str(CAR.TOYOTA_CAMRY_TSS3)})

    # Optional means "may be absent", not "ignore it": a present wrong EPS
    # identity must still reject the Camry even when the ABS identity matches.
    wrong_eps = bytearray(eps_version)
    wrong_eps[13] ^= 1
    mismatch = {(0x7A1, None): {bytes(wrong_eps)}, (0x7B0, None): {abs_version}}
    self.assertNotIn(str(CAR.TOYOTA_CAMRY_TSS3),
                     match_fw_to_car_exact(mismatch, match_brand="toyota", log=False))

    # Production control is not downgraded based on a transient diagnostic
    # response failure; runtime capability comes from source-real CAN state.
    self.assertEqual(self.CP.minSteerSpeed, 0.)
    self.assertTrue(self.CP.steerAtStandstill)

  def test_unified_tss3_request_and_result_dbc_layout(self):
    request_parser = CANParser("toyota_tss3_pt_generated", [("TSS3_CONTROL_REQUEST", 0)], 1)
    request_parser.update([(1_000_000_000, [CanData(0x08A, CAMRY_COMMON[0x08A], 1)])])
    request = request_parser.vl["TSS3_CONTROL_REQUEST"]
    self.assertAlmostEqual(request["LONGITUDINAL_REQUEST_ACCEL_A"], -0.442, places=6)
    self.assertAlmostEqual(request["LONGITUDINAL_REQUEST_ACCEL_B"], -0.442, places=6)
    self.assertEqual(request["DELAYED_HOLD_STATE"], 0)
    self.assertEqual(request["LONGITUDINAL_REQUEST_ID_A"], 11)
    self.assertEqual(request["LONGITUDINAL_ALLOCATION_METHOD_A"], 1)
    self.assertEqual(request["LONGITUDINAL_REQUEST_ID_B"], 17)
    self.assertEqual(request["LONGITUDINAL_ALLOCATION_METHOD_B"], 3)
    self.assertEqual(request["LATERAL_REQUEST_ID"], 0)
    self.assertAlmostEqual(request["LATERAL_REQUEST_PINION_ANGLE"], -0.203 * 1.000121519, places=6)
    self.assertAlmostEqual(request["LATERAL_ASSIST_GAIN"], 1.0, places=6)
    self.assertAlmostEqual(request["LATERAL_DAMPING_GAIN"], 0.0, places=6)
    self.assertEqual(request["REQUEST_SEQUENCE"], 60)

    # Retained relay-correct drive-A result frame with selected longitudinal ID11.
    result_frame = bytes.fromhex("00000018ffc80b730000000400000000000dffc8ffa2135dffa2000043d6390a")
    result_parser = CANParser("toyota_tss3_pt_generated", [("TSS3_CONTROL_RESULT", 0)], 1)
    result_parser.update([(1_010_000_000, [CanData(0x081, result_frame, 1)])])
    result = result_parser.vl["TSS3_CONTROL_RESULT"]
    self.assertEqual(result["LONGITUDINAL_RESULT_ID"], 11)
    self.assertEqual(result["REQUEST_LOSS_STATUS"], 0)
    self.assertEqual(result["LATERAL_RESULT_ID"], 0)
    self.assertAlmostEqual(result["LATERAL_RESULT_PINION_ANGLE"], 0.013 * 1.000121519, places=6)
    self.assertAlmostEqual(result["LONGITUDINAL_RESULT_ACCEL"], -0.094, places=6)

    state_parser = CANParser("toyota_tss3_pt_generated", [("TSS3_FRC_STATE_160", 0)], 2)
    state_parser.update([(1_020_000_000, [CanData(0x160, CAMRY_LONG, 2)])])
    frc_state = state_parser.vl["TSS3_FRC_STATE_160"]
    self.assertEqual(frc_state["COUNTER"], CAMRY_LONG[2])
    self.assertEqual(frc_state["BYTE_4"], CAMRY_LONG[4])
    self.assertEqual(frc_state["BYTE_5"], CAMRY_LONG[5])

    display_parser = CANParser("toyota_tss3_pt_generated", [("TSS3_CRUISE_DISPLAY", 0)], 1)
    display_parser.update([(1_030_000_000, [CanData(0x251, bytes.fromhex("a00000488088a080"), 1)])])
    self.assertEqual(display_parser.vl["TSS3_CRUISE_DISPLAY"]["SET_VEHICLE_INTERVAL_TIME"], 4)

  def test_stock_toyota_b_state_is_entirely_on_bus_one(self):
    ci = CarInterface(self.CP)
    self.assertEqual(ci.can_parsers[Bus.pt].bus, 1)
    state = update_state(ci)
    self.assertEqual(state.gearShifter, structs.CarState.GearShifter.drive)
    self.assertTrue(state.cruiseState.available)
    self.assertTrue(state.cruiseState.enabled)
    self.assertFalse(state.carNotReady)
    self.assertFalse(state.steerFaultTemporary)
    self.assertFalse(state.steerFaultPermanent)

  def test_delayed_hold_uses_request_id_and_allocation_not_raw_acc_state(self):
    ci = CarInterface(self.CP)
    normal = bytearray(CAMRY_COMMON[0x08A])
    normal[7] = 0x47  # request-B ID17 / Brake Only
    self.assertFalse(update_state(ci, control_request=bytes(normal)).cruiseState.standstill)

    hold = bytearray(CAMRY_COMMON[0x08A])
    hold[4] |= 0x20
    hold[7] = 0x67  # request-B ID25 / Brake Only
    self.assertTrue(update_state(ci, counter_offset=20, control_request=bytes(hold)).cruiseState.standstill)

    hold_override = bytearray(CAMRY_COMMON[0x08A])
    hold_override[4] |= 0x20
    hold_override[6:8] = bytes((0x2C, 0x66))  # A allocation0, B ID25/allocation2
    self.assertTrue(update_state(ci, counter_offset=40, control_request=bytes(hold_override)).cruiseState.standstill)

    moving_id25 = bytearray(CAMRY_COMMON[0x08A])
    moving_id25[6:8] = bytes((0x47, 0x65))  # retained moving counterexample: B ID25/allocation1
    self.assertFalse(update_state(ci, moving=True, counter_offset=60,
                                  control_request=bytes(moving_id25)).cruiseState.standstill)

  def test_current_fault_inhibit_asserts_and_clears_without_a_permanent_latch(self):
    ci = CarInterface(self.CP)
    clear = update_state(ci, moving=True, eps_status=0)
    self.assertFalse(clear.steerFaultTemporary)
    fault = update_state(ci, moving=True, counter_offset=20, eps_status=0x04)
    self.assertTrue(fault.steerFaultTemporary)
    self.assertFalse(fault.steerFaultPermanent)
    self.assertFalse(fault.vehicleSensorsInvalid)
    recovered = update_state(ci, moving=True, counter_offset=40, eps_status=0)
    self.assertFalse(recovered.steerFaultTemporary)
    self.assertFalse(recovered.steerFaultPermanent)

  def test_carstate_cooperative_inhibits_are_not_steering_faults(self):
    for command_inhibit, angle_inhibit in ((1, 0), (0, 1), (1, 1)):
      with self.subTest(command=command_inhibit, angle=angle_inhibit):
        ci = CarInterface(self.CP)
        raw = bytearray(CAMRY_COMMON[0x030])
        raw[16] = (raw[16] & ~1) | command_inhibit
        raw[19] = (raw[19] & ~1) | angle_inhibit
        state = update_state(ci, eps_telemetry=bytes(raw), hud=CAMRY_HUD)
        self.assertTrue(state.canValid)
        self.assertFalse(state.steerFaultTemporary)
        self.assertFalse(state.steerFaultPermanent)
        self.assertFalse(state.vehicleSensorsInvalid)

  def test_driver_override_with_cooperative_inhibit_uses_steering_pressed(self):
    # Measured override telemetry with F33_COOPERATIVE_COMMAND_INHIBIT set.
    override = bytes.fromhex("12000003330930b9130330053c800e99030b0000053c07b50000000042c3b381")
    state = update_state(CarInterface(self.CP), moving=True, eps_telemetry=override, hud=CAMRY_HUD)
    self.assertTrue(state.canValid)
    self.assertTrue(state.steeringPressed)
    self.assertFalse(state.steerFaultTemporary)
    self.assertFalse(state.steerFaultPermanent)

  def test_reference_initializing_source_does_not_report_a_steering_fault(self):
    # F33's reference-inhibit signal at B19[0] is stock cooperative state, not
    # an openpilot steering fault or a reason to surrender lateral ownership.
    initializing = bytes.fromhex("00000000170000500000100026820000000000010000ffff00000000b280595f")
    state = update_state(CarInterface(self.CP), eps_telemetry=initializing, hud=CAMRY_HUD)
    self.assertTrue(state.canValid)
    self.assertFalse(state.steerFaultTemporary)
    self.assertFalse(state.steerFaultPermanent)
    self.assertFalse(state.vehicleSensorsInvalid)

  def test_unrelated_eps_status_bits_are_not_promoted_to_faults(self):
    for status in (0, 1, 2, 8, 0xF0):
      with self.subTest(status=status):
        state = update_state(CarInterface(self.CP), eps_status=status)
        self.assertFalse(state.steerFaultTemporary)
        self.assertFalse(state.steerFaultPermanent)
        self.assertEqual(state.vehicleSensorsInvalid, bool(status & 1))

  def test_controller_emits_c7_without_unqualified_camry_longitudinal_output(self):
    ci = CarInterface(self.CP)
    update_state(ci, moving=True)
    output, sends = ci.apply(control(5.0, accel=1.2, long_active=True), 2_000_000_000)
    self.assertEqual(len(sends), 1)
    address, data, bus = sends[0]
    self.assertEqual((address, bus, len(data)), (0x777, 1, 8))
    self.assertEqual(data[:4], b"\x07\xC7\xC7\x01")
    self.assertEqual(data[6:], b"\x00\x00")
    self.assertAlmostEqual(output.steeringAngleDeg,
                           int.from_bytes(data[4:6], "big", signed=True) * (1024 / 17870), delta=0.03)
    self.assertEqual(output.accel, 0.0)
    self.assertFalse(any(address == 0x160 for address, _, _ in sends))

  def test_controller_emits_c7_at_native_100hz(self):
    ci = CarInterface(self.CP)
    update_state(ci, moving=True)
    sequences = []
    for i in range(4):
      _, sends = ci.apply(control(5.0), 2_000_000_000 + i * 10_000_000)
      self.assertEqual(len(sends), 1)
      address, data, bus = sends[0]
      self.assertEqual((address, bus), (0x777, 1))
      sequences.append(data[3])
    self.assertEqual(sequences, [1, 2, 3, 4])

  def test_host_request_plane_uses_normal_angle_control_without_emitting_c7(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)
    ci = CarInterface(cp)
    request = bytearray(CAMRY_COMMON[0x08A])
    request[21] = (request[21] & 0xC0) | 11
    state = update_state(ci, moving=True, control_request=bytes(request), bus=0, source_bus=2, hud=CAMRY_HUD)
    self.assertTrue(state.canValid)

    measured = state.steeringAngleDeg + state.steeringAngleOffsetDeg
    max_delta = CarControllerParams.F33_ANGLE_LIMITS.MAX_ANGLE_RATE
    output, sends = ci.apply(control(5.0), 2_000_000_000)
    self.assertFalse(any(address == 0x777 for address, _, _ in sends))
    self.assertAlmostEqual(output.steeringAngleDeg, max_delta, delta=0.01)

    # The asynchronous transport aligns the controller's existing limiter once
    # when the exact-clone handoff completes. It does not add an actuation gate.
    ci.CC.reset_tss3_lateral_target(measured)
    output, sends = ci.apply(control(5.0), 2_010_000_000)
    self.assertFalse(any(address == 0x777 for address, _, _ in sends))
    self.assertAlmostEqual(output.steeringAngleDeg, measured + max_delta, delta=0.01)

    # Toyota's currently selected source application is not a controller veto.
    request[21] = request[21] & 0xC0
    update_state(ci, moving=True, control_request=bytes(request), bus=0, source_bus=2, hud=CAMRY_HUD)
    output, sends = ci.apply(control(20.0), 2_020_000_000)
    self.assertFalse(any(address == 0x777 for address, _, _ in sends))
    self.assertGreater(output.steeringAngleDeg, measured + max_delta)
    self.assertAlmostEqual(output.steeringAngleDeg, measured + 2 * max_delta, delta=0.02)

  def test_host_request_plane_exposes_bounded_alpha_long_acceleration(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)
    ci = CarInterface(cp)
    update_state(ci, moving=True, bus=0, source_bus=2, hud=CAMRY_HUD)

    for requested, expected in ((1.2, 1.2), (2.0, 1.3), (-2.0, -1.5)):
      output, sends = ci.apply(control(0.0, active=False, accel=requested, long_active=True), 2_000_000_000)
      self.assertAlmostEqual(output.accel, expected)
      self.assertFalse(any(address == 0x08A for address, _, _ in sends))

    output, _ = ci.apply(control(0.0, active=False, accel=1.0, long_active=False), 2_010_000_000)
    self.assertEqual(output.accel, 0.0)

  def test_f33_uses_vehicle_model_limits_instead_of_tss2_rate_curve(self):
    ci = CarInterface(self.CP)
    state = update_state(ci, speed_ms=25.0)
    self.assertAlmostEqual(state.vEgoRaw, 25.0, delta=0.05)

    output, _ = ci.apply(control(20.0), 2_000_000_000)
    # The exact value comes from the Camry vehicle model's common lateral-jerk
    # envelope. It is deliberately above the inherited TSS2 0.075 deg/tick.
    self.assertGreater(output.steeringAngleDeg, 0.19)
    self.assertLess(output.steeringAngleDeg, 0.23)

  def test_host_request_plane_cancel_clones_native_brake_status_to_source_side(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)
    ci = CarInterface(cp)
    update_state(ci, bus=0, source_bus=2, hud=CAMRY_HUD)
    _, sends = ci.apply(control(0.0, active=False, cancel=True), 2_000_000_000)
    self.assertIn((0x101, bytes.fromhex("8800000100000093"), 2), sends)

  def test_inactive_c7_tracks_measured_angle_with_neutral_sequence(self):
    ci = CarInterface(self.CP)
    state = update_state(ci)
    _, sends = ci.apply(control(20.0, False), 2_000_000_000)
    _, data, bus = next(msg for msg in sends if msg[0] == 0x777)
    self.assertEqual(bus, 1)
    self.assertEqual(data[:4], b"\x07\xC7\xC7\x00")
    angle = int.from_bytes(data[4:6], "big", signed=True) * (1024 / 17870)
    self.assertAlmostEqual(angle, state.steeringAngleDeg, delta=0.12)

  def test_stock_harness_hud_is_observed_not_replaced(self):
    ci = CarInterface(self.CP)
    update_state(ci, hud=CAMRY_HUD)
    self.assertEqual(ci.can_parsers[Bus.pt].vl["TSS3_LKAS_HUD"]["BYTE_0"], 0x14)
    self.assertNotIn(0x412, ci.can_parsers[Bus.cam].addresses)
    for i in range(110):
      _, sends = ci.apply(control(1.0, cancel=True, left_lane=True, right_lane=True, steer_alert=True),
                          2_000_000_000 + i * 10_000_000)
      self.assertFalse(any(address in (0x101, 0x412) for address, _, _ in sends))

  def test_relay_hud_replaces_source_at_five_hz_and_on_alert_edges(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)
    ci = CarInterface(cp)
    disabled_hud = bytes.fromhex("1000002200ee9307")
    update_state(ci, bus=0, source_bus=2, hud=disabled_hud)

    _, sends = ci.apply(control(1.0, left_lane=True, right_lane=True), 2_000_000_000)
    self.assertIn((0x412, bytes.fromhex("1400004401ee9307"), 0), sends)

    for i in range(1, 20):
      _, sends = ci.apply(control(1.0, left_lane=True, right_lane=True), 2_000_000_000 + i * 10_000_000)
      self.assertFalse(any(address == 0x412 for address, _, _ in sends))
    _, sends = ci.apply(control(1.0, left_lane=True, right_lane=True), 2_200_000_000)
    self.assertIn((0x412, bytes.fromhex("1400004401ee9307"), 0), sends)

    _, sends = ci.apply(control(1.0, left_lane=True, right_lane=True, steer_alert=True), 2_210_000_000)
    self.assertIn((0x412, bytes.fromhex("140c004401ee9307"), 0), sends)
    _, sends = ci.apply(control(1.0, left_lane=True, right_lane=True), 2_220_000_000)
    self.assertIn((0x412, bytes.fromhex("1400004401ee9307"), 0), sends)

  def test_gap_state_maps_to_absolute_bars_and_lta_exposes_button_event(self):
    ci = CarInterface(self.CP)
    state = update_state(ci, hud=bytes.fromhex("1200002202ee9307"))
    self.assertEqual(state.cruiseState.followDistanceBars, 4)

    distance = bytearray(CAMRY_COMMON[0x251])
    distance[5] = (distance[5] & 0x1F) | (2 << 5)
    state = update_state(ci, counter_offset=20, hud=bytes.fromhex("1200002202ee9307"),
                         cruise_display=bytes(distance), iterations=1)
    self.assertEqual(state.cruiseState.followDistanceBars, 3)
    self.assertEqual(list(state.buttonEvents), [])

    state = update_state(ci, counter_offset=21, hud=bytes.fromhex("1000002200ee9307"),
                         cruise_display=bytes(distance), iterations=1)
    self.assertEqual([(event.type, event.pressed) for event in state.buttonEvents], [
      (structs.CarState.ButtonEvent.Type.lkas, True),
      (structs.CarState.ButtonEvent.Type.lkas, False),
    ])

  def test_reengagement_does_not_reuse_the_residents_consumed_sequence(self):
    ci = CarInterface(self.CP)
    update_state(ci)
    frames = []
    for active in (True, False, True):
      _, sends = ci.apply(control(1.0, active=active), 2_000_000_000)
      frames.append(next(data for address, data, _ in sends if address == 0x777))
      ci.apply(control(1.0, active=active), 2_010_000_000)
    self.assertEqual([frame[3] for frame in frames], [1, 0, 3])


class TestToyotaCamryTSS3Safety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    param = EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.F33 | ToyotaSafetyFlags.STOCK_LONGITUDINAL
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()
    for address in (0x025, 0x0AA, 0x116, 0x101, 0x08A):
      self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, 1, CAMRY_COMMON[address])))
    self.assertTrue(self.safety.get_controls_allowed())

  @staticmethod
  def c7(angle_raw: int = 0, sequence: int = 1, bus: int = 1):
    data = b"\x07\xC7\xC7" + bytes((sequence,)) + angle_raw.to_bytes(2, "big", signed=True) + b"\x00\x00"
    return libsafety_py.make_CANPacket(0x777, bus, data)

  def set_speed(self, speed_ms: float):
    wheel_raw = 6767 + round(speed_ms * 3.6 / 0.01)
    msg = libsafety_py.make_CANPacket(0x0AA, 1, wheel_raw.to_bytes(2, "big") * 4)
    for _ in range(6):
      self.assertTrue(self.safety.safety_rx_hook(msg))

  def test_accepts_bounded_c7_only_on_unsplit_bus(self):
    self.assertTrue(self.safety.safety_tx_hook(self.c7()))
    self.assertFalse(self.safety.safety_tx_hook(self.c7(bus=0)))
    # 0x777 remains a diagnostic address in stock firmware; Panda grants only
    # the exact private C7 envelope, never arbitrary functional diagnostics.
    self.assertFalse(self.safety.safety_tx_hook(
      libsafety_py.make_CANPacket(0x777, 1, bytes.fromhex("0210030000000000"))))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x08A, 1, CAMRY_COMMON[0x08A])))

  def test_0x160_is_not_host_replaceable(self):
    for bus in range(3):
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x160, bus, CAMRY_LONG)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x160), 0)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x230), 0)

  def test_rejects_bad_header_reserved_bytes_and_overangle(self):
    for index in (0, 1, 2, 6, 7):
      data = bytearray(self.c7()[0].data)
      data[index] ^= 1
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x777, 1, bytes(data))))

    self.assertFalse(self.safety.safety_tx_hook(self.c7(1746)))

  def test_stock_cruise_latch_owns_controls_allowed(self):
    disabled = bytearray(CAMRY_COMMON[0x08A])
    disabled[3] &= ~0x08
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 1, bytes(disabled))))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(self.c7()))

  def test_unsplit_hud_and_brake_are_not_host_replaceable(self):
    for address, data in ((0x101, bytes.fromhex("8800000600000098")),
                          (0x412, bytes.fromhex("1400004401ee9307"))):
      for bus in range(3):
        self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data)))
      self.assertEqual(self.safety.safety_fwd_hook(2, address), 0)

  def test_absolute_angle_limit_is_not_just_a_rate_limit(self):
    for sign in (-1, 1):
      self.safety.set_controls_allowed(True)
      self.safety.set_desired_angle_last(sign * 1744)
      self.assertTrue(self.safety.safety_tx_hook(self.c7(sign * 1745)))
      self.safety.set_controls_allowed(True)
      self.safety.set_desired_angle_last(sign * 1745)
      self.assertFalse(self.safety.safety_tx_hook(self.c7(sign * 1746)))

  def test_f33_conditioner_and_vehicle_model_rate_limits(self):
    # At a standstill the recovered F33 conditioner is the tighter bound:
    # seven B6 counts per 10 ms controller tick.
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    self.assertTrue(self.safety.safety_tx_hook(self.c7(7)))
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    self.assertFalse(self.safety.safety_tx_hook(self.c7(8)))

    # At highway speed the common vehicle-model lateral-jerk envelope is
    # tighter than the actuator conditioner and uses the exact Camry geometry.
    self.setUp()
    self.set_speed(25.0)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    self.assertTrue(self.safety.safety_tx_hook(self.c7(4)))
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    self.assertFalse(self.safety.safety_tx_hook(self.c7(5)))

  def test_relay_correct_host_mode_uses_bus0_state_parser(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], True, False, False)
    cp.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.TSS3_08A_HOST.value
    ci = CarInterface(cp)
    self.assertEqual(ci.can_parsers[Bus.pt].bus, 0)
    self.assertEqual(ci.can_parsers[Bus.cam].bus, 2)
    state = update_state(ci, bus=0, source_bus=2, hud=CAMRY_HUD)
    self.assertTrue(state.canValid)
    self.assertTrue(state.standstill)

    # Missing either side of the physical split invalidates CarState.
    ci_missing_source = CarInterface(cp)
    state_missing_source = update_state(ci_missing_source, bus=0, hud=CAMRY_HUD)
    self.assertFalse(state_missing_source.canValid)
    ci_wrong_state = CarInterface(cp)
    state_wrong_state = update_state(ci_wrong_state, bus=1, source_bus=2, hud=CAMRY_HUD)
    self.assertFalse(state_wrong_state.canValid)


class TestToyotaCamryTSS3RequestReplacementSafety(unittest.TestCase):
  PARAM = (EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.F33 |
           ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.TSS3_08A_HOST)
  ALPHA_LONG_PARAM = PARAM & ~ToyotaSafetyFlags.STOCK_LONGITUDINAL

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, self.PARAM), 0)
    self.safety.init_tests()
    self.safety.set_timer(0)

  @staticmethod
  def source_08a(b26: int = 0x12, *, target_id: int = 0, angle_raw: int = 0,
                 semantic: int | None = None, fv4: int | None = None,
                 request_a: int | None = None, request_b: int | None = None):
    data = bytearray(CAMRY_COMMON[0x08A])
    if request_a is not None:
      data[6] = request_a
    if request_b is not None:
      data[7] = request_b
    data[18:20] = angle_raw.to_bytes(2, "big", signed=True)
    data[21] = (data[21] & 0xC0) | (target_id & 0x3F)
    if semantic is not None:
      data[22] = semantic
    data[26] = (data[26] & 0xC0) | (b26 & 0x3F)
    if fv4 is not None:
      data[28] = ((fv4 & 0xF) << 4) | (data[28] & 0x0F)
    msg = libsafety_py.make_CANPacket(0x08A, 2, bytes(data))
    msg[0].fd = 1
    return msg

  @staticmethod
  def admin(action: int):
    return libsafety_py.make_CANPacket(0x777, 1, bytes((7, 0xC9, 0xA8, action, 0, 0, 0, 0)))

  @staticmethod
  def sync(reset: int = 0x12345, trip: int = 0x026C):
    data = bytearray(8)
    data[0:2] = trip.to_bytes(2, "big")
    data[2] = (reset >> 12) & 0xFF
    data[3] = (reset >> 4) & 0xFF
    data[4] = (reset & 0xF) << 4
    return libsafety_py.make_CANPacket(0x00F, 0, bytes(data))

  @staticmethod
  def c7(angle_raw: int = 0, sequence: int = 1):
    data = b"\x07\xC7\xC7" + bytes((sequence,)) + angle_raw.to_bytes(2, "big", signed=True) + b"\x00\x00"
    return libsafety_py.make_CANPacket(0x777, 1, data)

  @staticmethod
  def cruise_switch(button: str | None = None):
    data = bytearray(32)
    if button == "resume":
      data[3] |= 0x80
    elif button == "set":
      data[4] |= 0x80
    elif button == "cancel":
      data[4] |= 0x40
    elif button == "main":
      data[7] |= 0x04
    msg = libsafety_py.make_CANPacket(0x0FE, 0, bytes(data))
    msg[0].fd = 1
    return msg

  @staticmethod
  def oracle_fragment(fragment: int, seq: int = 1, fill: int = 0):
    header = ((fragment & 0x7) << 5) | (seq & 0x1F)
    if fragment == 4:
      data = bytes((header, 0x08, 0x55, 0xC9, 0xA8, seq ^ 0xFF, 0x5A, 0xA5))
    else:
      data = bytes((header, fill, fill, fill, fill, fill, fill, fill))
    return libsafety_py.make_CANPacket(0x1FDC0002, 0, data)

  @staticmethod
  def host_frame(source: bytes, *, angle_raw: int | None = None, target_id: int | None = None,
                 assist_gain_raw: int | None = None, accel_a: int | None = None,
                 accel_b: int | None = None, request_a: int | None = None,
                 request_b: int | None = None, mutate_mac: bool = False):
    data = bytearray(source)
    if angle_raw is not None:
      data[18:20] = angle_raw.to_bytes(2, "big", signed=True)
    if target_id is not None:
      data[21] = (data[21] & 0xC0) | (target_id & 0x3F)
    if assist_gain_raw is not None:
      data[24] = assist_gain_raw
    if request_a is not None:
      data[6] = request_a
    if request_b is not None:
      data[7] = request_b
    if accel_a is not None:
      data[8:10] = accel_a.to_bytes(2, "big", signed=True)
    if accel_b is not None:
      data[11:13] = accel_b.to_bytes(2, "big", signed=True)
    if mutate_mac:
      data[28] ^= 0x0F  # MAC28 only; preserve FV4 high nibble
      data[29] ^= 0xA5
      data[30] ^= 0x5A
      data[31] ^= 0xFF
    msg = libsafety_py.make_CANPacket(0x08A, 0, bytes(data))
    msg[0].fd = 1
    return msg

  def observe_source(self, **kwargs) -> bytes:
    msg = self.source_08a(**kwargs)
    self.assertTrue(self.safety.safety_rx_hook(msg))
    return bytes(msg[0].data)[:32]

  def set_speed(self, speed_ms: float):
    wheel_raw = 6767 + round(speed_ms * 3.6 / 0.01)
    msg = libsafety_py.make_CANPacket(0x0AA, 0, wheel_raw.to_bytes(2, "big") * 4)
    for _ in range(6):
      self.assertTrue(self.safety.safety_rx_hook(msg))

  def arm(self):
    self.assertTrue(self.safety.safety_tx_hook(self.admin(1)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

  def use_alpha_long_safety(self):
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, self.ALPHA_LONG_PARAM), 0)
    self.safety.init_tests()
    self.safety.set_timer(0)

  def test_relay_open_rx_checks_require_native_sources_not_forwarded_bus0_08a(self):
    def fd(addr: int, bus: int, data: bytes):
      msg = libsafety_py.make_CANPacket(addr, bus, data)
      msg[0].fd = 1
      return msg

    # Real relay-open topology: state/chassis sources are native bus0, while
    # protected 0x08A is native bus2. There is deliberately no bus0 RX 0x08A;
    # that downstream copy is produced by Panda forwarding/host replacement.
    self.assertTrue(self.safety.safety_rx_hook(fd(0x025, 0, CAMRY_COMMON[0x025])))
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x0AA, 0, CAMRY_COMMON[0x0AA])))
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x116, 0, CAMRY_COMMON[0x116])))
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x101, 0, CAMRY_COMMON[0x101])))
    self.assertTrue(self.safety.safety_rx_hook(self.source_08a()))
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x00F, 0, bytes(8))))
    self.assertTrue(self.safety.safety_config_valid())

  def test_relay_open_native_08a_owns_controls_allowed(self):
    self.assertFalse(self.safety.get_controls_allowed())

    enabled = self.source_08a(target_id=11)
    self.assertTrue(self.safety.safety_rx_hook(enabled))
    self.assertTrue(self.safety.get_controls_allowed())

    disabled = bytearray(bytes(enabled[0].data)[:32])
    disabled[3] &= ~0x08
    disabled_msg = libsafety_py.make_CANPacket(0x08A, 2, bytes(disabled))
    disabled_msg[0].fd = 1
    self.assertTrue(self.safety.safety_rx_hook(disabled_msg))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_alpha_long_controls_allowed_follows_native_cruise_latch(self):
    self.use_alpha_long_safety()

    self.assertTrue(self.safety.safety_rx_hook(self.source_08a(target_id=11)))
    self.assertTrue(self.safety.get_controls_allowed())

    # Button bits do not own a second engagement state. FRC reflects their
    # accepted result in the following source-real operating latch.
    for button in ("set", "resume", "cancel", "main"):
      self.assertTrue(self.safety.safety_rx_hook(self.cruise_switch(button)))
      self.assertTrue(self.safety.get_controls_allowed())

    disabled = self.source_08a(target_id=0)
    disabled[0].data[3] &= ~0x08
    self.assertTrue(self.safety.safety_rx_hook(disabled))
    self.assertFalse(self.safety.get_controls_allowed())

    for button in ("set", "resume", "cancel", "main"):
      self.assertTrue(self.safety.safety_rx_hook(self.cruise_switch(button)))
      self.assertFalse(self.safety.get_controls_allowed())

    self.assertTrue(self.safety.safety_rx_hook(self.source_08a(target_id=11)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_alpha_long_button_bit_does_not_preempt_source_latch(self):
    self.use_alpha_long_safety()

    handoff = self.observe_source(target_id=0, b26=0x20, request_a=0x00, request_b=0x12)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    self.assertTrue(self.safety.get_controls_allowed())

    source = self.observe_source(target_id=0, b26=0x21, request_a=0x00, request_b=0x12)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(
      source, target_id=11, assist_gain_raw=100, request_a=0x2D, request_b=0x47,
      accel_a=-500, accel_b=-500, mutate_mac=True)))

    # The raw button cannot independently revoke the request plane. The next
    # source-real operating latch is the shared engagement result.
    self.assertTrue(self.safety.safety_rx_hook(self.cruise_switch("main")))
    self.assertTrue(self.safety.get_controls_allowed())
    next_source = self.observe_source(target_id=0, b26=0x22, request_a=0x00, request_b=0x12)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(
      next_source, target_id=11, assist_gain_raw=100, request_a=0x2D, request_b=0x47,
      accel_a=-500, accel_b=-500, mutate_mac=True)))

  def test_camry_brake_cancel_safety_is_stock_shaped_and_checksum_valid(self):
    good = bytes.fromhex("8800000100000093")
    self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, good)))

    brake_off = bytearray(good)
    brake_off[0] &= ~0x08
    brake_off[7] = (0x01 + 0x01 + 8 + sum(brake_off[:7])) & 0xFF
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, bytes(brake_off))))

    bad_shape = bytearray(good)
    bad_shape[4] = 1
    bad_shape[7] = (0x01 + 0x01 + 8 + sum(bad_shape[:7])) & 0xFF
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, bytes(bad_shape))))

    bad_checksum = bytearray(good)
    bad_checksum[7] ^= 1
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, bytes(bad_checksum))))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 0, good)))

  def test_camry_hud_is_classic_bus0_replacement(self):
    hud = libsafety_py.make_CANPacket(0x412, 0, bytes.fromhex("1400004401ee9307"))
    self.assertTrue(self.safety.safety_tx_hook(hud))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x412), -1)

    fd_hud = libsafety_py.make_CANPacket(0x412, 0, bytes.fromhex("1400004401ee9307"))
    fd_hud[0].fd = 1
    self.assertFalse(self.safety.safety_tx_hook(fd_hud))
    self.assertFalse(self.safety.safety_tx_hook(
      libsafety_py.make_CANPacket(0x412, 2, bytes.fromhex("1400004401ee9307"))))

  def test_oracle_transport_is_one_ordered_raw_classic_transaction(self):
    # Continuations are never valid without a fresh fragment zero.
    self.assertFalse(self.safety.safety_tx_hook(self.oracle_fragment(1, 9)))

    for fragment in range(5):
      self.assertTrue(self.safety.safety_tx_hook(self.oracle_fragment(fragment, 9, fill=fragment)))

    # Completion closes the transaction; only a new fragment zero can restart it.
    self.assertFalse(self.safety.safety_tx_hook(self.oracle_fragment(1, 9)))
    self.assertTrue(self.safety.safety_tx_hook(self.oracle_fragment(0, 10)))
    self.assertFalse(self.safety.safety_tx_hook(self.oracle_fragment(1, 11)))

    # Fragment zero is an explicit restart after any partial/malformed sequence.
    self.assertTrue(self.safety.safety_tx_hook(self.oracle_fragment(0, 11)))
    for fragment in range(1, 4):
      self.assertTrue(self.safety.safety_tx_hook(self.oracle_fragment(fragment, 11, fill=fragment)))
    bad_tail = bytearray(self.oracle_fragment(4, 11)[0].data)
    bad_tail[6] ^= 1
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x1FDC0002, 0, bytes(bad_tail))))
    self.assertTrue(self.safety.safety_tx_hook(self.oracle_fragment(0, 12)))

    # No FD form, wrong bus, zero sequence, or legacy diagnostic carrier is admitted.
    fd = self.oracle_fragment(0, 13)
    fd[0].fd = 1
    self.assertFalse(self.safety.safety_tx_hook(fd))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x1FDC0002, 1, bytes(self.oracle_fragment(0, 13)[0].data))))
    self.assertFalse(self.safety.safety_tx_hook(self.oracle_fragment(0, 0)))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x7A1, 0, bytes.fromhex("1028c9c901008a00"))))

  def test_arm_is_fresh_source_ownership_not_id_or_motion_policy(self):
    self.assertFalse(self.safety.safety_tx_hook(self.admin(1)))

    # Any fresh native request identity can establish relay ownership; the arm
    # itself carries no steering authority.
    self.observe_source(target_id=18)
    self.assertTrue(self.safety.safety_tx_hook(self.admin(1)))
    self.assertTrue(self.safety.safety_tx_hook(self.admin(0)))

    self.observe_source(target_id=11)
    moving = libsafety_py.make_CANPacket(0x0AA, 0, bytes.fromhex("1c001c001c001c00"))
    self.assertTrue(self.safety.safety_rx_hook(moving))
    self.assertTrue(self.safety.safety_tx_hook(self.admin(1)))

    self.assertTrue(self.safety.safety_tx_hook(self.admin(0)))
    self.safety.set_timer(100_001)
    self.assertFalse(self.safety.safety_tx_hook(self.admin(1)))

  def test_first_handoff_clone_seeds_angle_rate_from_measured_steering(self):
    # init_tests() leaves the sampled measured steering baseline at zero. The
    # first exact handoff clone must keep that measured baseline instead of
    # adopting Toyota's unrelated native ID11 request angle.
    source = self.observe_source(target_id=11, angle_raw=50, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(source)))
    self.assertEqual(self.safety.get_desired_angle_last(), 0)

    # While the shared request plane remains owned for the other axis, lateral
    # is explicitly inactive rather than passing Toyota's ID11 through.
    next_source = self.observe_source(target_id=11, angle_raw=0, b26=0x21)
    self.safety.set_controls_allowed(False)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(
      next_source, angle_raw=0, target_id=0, assist_gain_raw=100, mutate_mac=True)))
    self.assertEqual(self.safety.get_desired_angle_last(), 0)

  def test_request_plane_allows_four_vehicle_model_controller_deltas(self):
    handoff = self.observe_source(target_id=11, angle_raw=0, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    self.set_speed(25.0)

    source = self.observe_source(target_id=11, angle_raw=0, b26=0x21)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(source, angle_raw=16, mutate_mac=True)))

    next_source = self.observe_source(target_id=11, angle_raw=0, b26=0x22)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    self.assertFalse(self.safety.safety_tx_hook(self.host_frame(next_source, angle_raw=17, mutate_mac=True)))

  def test_lateral_rate_reject_drops_one_generation_and_recovers_in_place(self):
    handoff = self.observe_source(target_id=11, angle_raw=-551, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(-551)

    # This is the road fault: an inactive/active steering transition produced
    # 29 B6 counts against the 28-count sampled-controller envelope. Normal
    # angle safety rejects the one command and rebases to measured steering.
    rejected_source = self.observe_source(target_id=11, angle_raw=-551, b26=0x21)
    self.assertFalse(self.safety.safety_tx_hook(
      self.host_frame(rejected_source, angle_raw=-522, mutate_mac=True)))
    self.assertEqual(self.safety.get_desired_angle_last(), 0)

    # Like every other Panda-controlled angle car, a safety reject does not
    # relinquish message ownership. The source generation is consumed and the
    # next ordinary bounded command recovers from the measured-angle baseline.
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)
    next_source = self.observe_source(target_id=11, angle_raw=-522, b26=0x22)
    self.assertTrue(self.safety.safety_tx_hook(
      self.host_frame(next_source, angle_raw=7, mutate_mac=True)))
    self.assertEqual(self.safety.get_desired_angle_last(), 7)

  def test_host_must_consume_native_generations_oldest_first(self):
    handoff = self.observe_source(target_id=0, angle_raw=-110, b26=0x1F, semantic=0x5F, fv4=7)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))

    oldest = self.observe_source(target_id=0, angle_raw=-100, b26=0x20, semantic=0x60, fv4=8)
    middle = self.observe_source(target_id=0, angle_raw=-95, b26=0x21, semantic=0x61, fv4=9)
    newest = self.observe_source(target_id=0, angle_raw=-90, b26=0x22, semantic=0x62, fv4=10)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(-110)

    # A newer source-real generation cannot skip older unconsumed generations.
    newest_host = self.host_frame(newest, angle_raw=-85, target_id=11, assist_gain_raw=100, mutate_mac=True)
    self.assertFalse(self.safety.safety_tx_hook(newest_host))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

    self.setUp()
    handoff = self.observe_source(target_id=0, angle_raw=-110, b26=0x1F, semantic=0x5F, fv4=7)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    oldest = self.observe_source(target_id=0, angle_raw=-100, b26=0x20, semantic=0x60, fv4=8)
    middle = self.observe_source(target_id=0, angle_raw=-95, b26=0x21, semantic=0x61, fv4=9)
    newest = self.observe_source(target_id=0, angle_raw=-90, b26=0x22, semantic=0x62, fv4=10)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(-110)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(oldest, angle_raw=-108, target_id=11, assist_gain_raw=100, mutate_mac=True)))
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(middle, angle_raw=-106, target_id=11, assist_gain_raw=100, mutate_mac=True)))
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(newest, angle_raw=-104, target_id=11, assist_gain_raw=100, mutate_mac=True)))

  def test_exact_clone_preserves_non_id11_requests_and_is_single_use(self):
    source = self.observe_source(target_id=18, b26=0x12, semantic=0x51)
    self.arm()
    clone = self.host_frame(source)
    self.safety.set_controls_allowed(False)
    self.assertTrue(self.safety.safety_tx_hook(clone))

    # A source generation is consumed once. Replay is rejected and fails open.
    self.assertFalse(self.safety.safety_tx_hook(clone))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

  def test_unknown_toyota_intervention_id_remains_source_exact(self):
    handoff = self.observe_source(target_id=0, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))

    intervention = self.observe_source(target_id=1, angle_raw=100, b26=0x21)
    self.safety.set_controls_allowed(False)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(intervention)))

    changed = self.observe_source(target_id=1, angle_raw=100, b26=0x22)
    self.assertFalse(self.safety.safety_tx_hook(self.host_frame(changed, angle_raw=101, mutate_mac=True)))

  def test_native_id0_can_promote_to_id11_with_lta_gain(self):
    handoff = self.observe_source(target_id=0, angle_raw=-100, b26=0x20, semantic=0x5F, fv4=7)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    source = self.observe_source(target_id=0, angle_raw=-100, b26=0x21, semantic=0x60, fv4=8)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(-100)

    promoted = self.host_frame(source, angle_raw=-95, target_id=11, assist_gain_raw=100, mutate_mac=True)
    self.assertTrue(self.safety.safety_tx_hook(promoted))

    # Promoted ID0 must use Toyota's LTA/LCA assist gain; arbitrary gain is rejected.
    next_source = self.observe_source(target_id=0, angle_raw=-95, b26=0x22, semantic=0x61, fv4=9)
    wrong_gain = self.host_frame(next_source, angle_raw=-90, target_id=11, assist_gain_raw=0, mutate_mac=True)
    self.assertFalse(self.safety.safety_tx_hook(wrong_gain))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

    # Any companion edit outside B18:B19/B21-low6/B24=100/MAC remains forbidden.
    next_source = self.observe_source(target_id=0, angle_raw=-90, b26=0x23, semantic=0x62, fv4=10)
    bad = bytearray(next_source)
    bad[18:20] = (-90).to_bytes(2, "big", signed=True)
    bad[21] = (bad[21] & 0xC0) | 11
    bad[24] = 100
    bad[28] ^= 0x0F
    bad[29] ^= 0xA5
    bad[30] ^= 0x5A
    bad[31] ^= 0xFF
    bad[7] ^= 1
    bad_msg = libsafety_py.make_CANPacket(0x08A, 0, bytes(bad))
    bad_msg[0].fd = 1
    self.assertFalse(self.safety.safety_tx_hook(bad_msg))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

  def test_modified_frame_is_id11_angle_only(self):
    handoff = self.observe_source(target_id=11, angle_raw=100, b26=0x21, semantic=0x60, fv4=8)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    source = self.observe_source(target_id=11, angle_raw=100, b26=0x22, semantic=0x61, fv4=9)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(100)

    modified = self.host_frame(source, angle_raw=105, mutate_mac=True)
    self.assertTrue(self.safety.safety_tx_hook(modified))

    # ID, sequence, gains, longitudinal fields, and every other application byte
    # remain source-real. Any non-angle semantic edit is rejected.
    next_source = self.observe_source(target_id=11, angle_raw=105, b26=0x23, semantic=0x62, fv4=10)
    bad = bytearray(self.host_frame(next_source, angle_raw=110, mutate_mac=True)[0].data)[:32]
    bad[7] ^= 1
    bad_msg = libsafety_py.make_CANPacket(0x08A, 0, bytes(bad))
    bad_msg[0].fd = 1
    self.assertFalse(self.safety.safety_tx_hook(bad_msg))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

  def test_id4_and_id18_can_be_replaced_or_explicitly_disabled(self):
    handoff = self.observe_source(target_id=11, angle_raw=100, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(100)

    for b26, native_id in ((0x21, 4), (0x22, 18)):
      self.safety.set_desired_angle_last(100)
      source = self.observe_source(target_id=native_id, angle_raw=100, b26=b26)
      promoted = self.host_frame(source, angle_raw=102, target_id=11, assist_gain_raw=100, mutate_mac=True)
      self.assertTrue(self.safety.safety_tx_hook(promoted))
      self.safety.set_desired_angle_last(102)

      # Longitudinal-only operation commands the measured inactive angle and
      # ID0 instead of preserving a stock lateral request.
      next_source = self.observe_source(target_id=native_id, angle_raw=102, b26=b26 + 2)
      self.assertTrue(self.safety.safety_tx_hook(self.host_frame(
        next_source, angle_raw=0, target_id=0, assist_gain_raw=100, mutate_mac=True)))

  def test_alpha_long_promotes_longitudinal_ids_and_replaces_equal_bounds(self):
    self.use_alpha_long_safety()
    handoff = self.observe_source(target_id=0, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    self.safety.set_controls_allowed(True)

    source = self.observe_source(target_id=0, b26=0x21)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(source, accel_a=-500, accel_b=-500, mutate_mac=True)))

    idle = self.observe_source(target_id=0, b26=0x22, request_a=0x00, request_b=0x12)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(
      idle, request_a=0x2D, request_b=0x47, accel_a=1300, accel_b=1300, mutate_mac=True)))

  def test_alpha_long_gas_override_drops_stale_generation_without_releasing(self):
    self.use_alpha_long_safety()
    handoff = self.observe_source(target_id=11, angle_raw=0, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)

    stale_source = self.observe_source(target_id=11, angle_raw=0, b26=0x21)
    self.safety.set_gas_pressed_prev(True)
    self.assertFalse(self.safety.safety_tx_hook(
      self.host_frame(stale_source, angle_raw=16, accel_a=-500, accel_b=-500, mutate_mac=True)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)
    self.assertEqual(self.safety.get_desired_angle_last(), 16)

    # A second command may already have been created before card observes the
    # gas edge. It is dropped on the longitudinal axis while its valid lateral
    # progression remains the baseline for the next combined generation.
    next_stale_source = self.observe_source(target_id=11, angle_raw=0, b26=0x22)
    self.safety.set_gas_pressed_prev(True)
    self.assertFalse(self.safety.safety_tx_hook(
      self.host_frame(next_stale_source, angle_raw=28, accel_a=-500, accel_b=-500, mutate_mac=True)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)
    self.assertEqual(self.safety.get_desired_angle_last(), 28)

    # Toyota changes the allocation companions to 0/2 while the driver gas
    # override is active. It is still the ordinary ID11/ID17 request and must
    # become openpilot's normal inactive command instead of leaking alternating
    # native positive acceleration into the owned request plane.
    inactive_msg = self.source_08a(target_id=11, angle_raw=0, b26=0x23,
                                   request_a=0x2C, request_b=0x46)
    inactive_data = bytearray(bytes(inactive_msg[0].data)[:32])
    inactive_data[8:10] = (714).to_bytes(2, "big", signed=True)
    inactive_data[11:13] = (714).to_bytes(2, "big", signed=True)
    inactive_msg = libsafety_py.make_CANPacket(0x08A, 2, bytes(inactive_data))
    inactive_msg[0].fd = 1
    self.assertTrue(self.safety.safety_rx_hook(inactive_msg))
    inactive_source = bytes(inactive_msg[0].data)[:32]
    self.assertTrue(self.safety.safety_tx_hook(
      self.host_frame(inactive_source, angle_raw=40, request_a=0x2D, request_b=0x47,
                      accel_a=0, accel_b=0, mutate_mac=True)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

  def test_alpha_long_rejects_unequal_or_out_of_range_bounds(self):
    for accel_a, accel_b in ((-500, -499), (-1501, -1501), (1301, 1301)):
      with self.subTest(accel_a=accel_a, accel_b=accel_b):
        self.use_alpha_long_safety()
        handoff = self.observe_source(target_id=0, b26=0x20)
        self.arm()
        self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
        self.safety.set_controls_allowed(True)
        source = self.observe_source(target_id=0, b26=0x21)
        self.assertFalse(self.safety.safety_tx_hook(
          self.host_frame(source, accel_a=accel_a, accel_b=accel_b, mutate_mac=True)))

  def test_alpha_long_preserves_alternate_intervention_tuple(self):
    self.use_alpha_long_safety()
    handoff = self.observe_source(target_id=0, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    self.safety.set_controls_allowed(True)

    intervention = self.observe_source(target_id=0, b26=0x21, request_a=0x35, request_b=0x53)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(intervention, mutate_mac=True)))

    changed = self.observe_source(target_id=0, b26=0x22, request_a=0x35, request_b=0x53)
    self.assertFalse(self.safety.safety_tx_hook(
      self.host_frame(changed, accel_a=-500, accel_b=-500, mutate_mac=True)))

  def test_alpha_long_reclaims_delayed_hold_request_owner(self):
    self.use_alpha_long_safety()
    handoff = self.observe_source(target_id=0, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    self.safety.set_controls_allowed(True)

    delayed_hold_msg = self.source_08a(target_id=0, b26=0x21, request_a=0x2D, request_b=0x67)
    delayed_hold_msg[0].data[4] |= 0x20
    self.assertTrue(self.safety.safety_rx_hook(delayed_hold_msg))
    delayed_hold = bytes(delayed_hold_msg[0].data)[:32]
    resumed = bytearray(delayed_hold)
    resumed[4] &= ~0x20
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(
      bytes(resumed), request_b=0x47, accel_a=-500, accel_b=-500, mutate_mac=True)))

  def test_stock_longitudinal_mode_rejects_acceleration_replacement(self):
    handoff = self.observe_source(target_id=0, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    self.safety.set_controls_allowed(True)
    source = self.observe_source(target_id=0, b26=0x21)
    self.assertFalse(self.safety.safety_tx_hook(
      self.host_frame(source, accel_a=-500, accel_b=-500, mutate_mac=True)))

  def test_alpha_long_cannot_synthesize_unowned_cruise_state(self):
    for byte_index, bit_mask in ((3, 0x08), (4, 0x10), (20, 0xC0), (22, 0x10)):
      with self.subTest(byte_index=byte_index, bit_mask=bit_mask):
        self.use_alpha_long_safety()
        handoff = self.observe_source(target_id=0, b26=0x20)
        self.arm()
        self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
        self.safety.set_controls_allowed(True)
        source = self.observe_source(target_id=0, b26=0x21)
        bad = bytearray(self.host_frame(source, accel_a=-500, accel_b=-500, mutate_mac=True)[0].data)[:32]
        bad[byte_index] ^= bit_mask
        bad_msg = libsafety_py.make_CANPacket(0x08A, 0, bytes(bad))
        bad_msg[0].fd = 1
        self.assertFalse(self.safety.safety_tx_hook(bad_msg))

  def test_modified_non_id11_is_rejected(self):
    handoff = self.observe_source(target_id=11, angle_raw=100, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    source = self.observe_source(target_id=18, angle_raw=100, b26=0x21)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(100)
    self.assertFalse(self.safety.safety_tx_hook(self.host_frame(source, angle_raw=101, target_id=18, mutate_mac=True)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

  def test_source_exact_id11_still_requires_controls_allowed(self):
    handoff = self.observe_source(target_id=11, angle_raw=100, b26=0x20)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    source = self.observe_source(target_id=11, angle_raw=100, b26=0x21)
    self.safety.set_controls_allowed(False)
    self.safety.set_desired_angle_last(100)
    self.assertFalse(self.safety.safety_tx_hook(self.host_frame(source)))

  def test_latest_00f_cannot_veto_a_matched_native_08a_generation(self):
    old_reset = 0x12345
    new_reset = old_reset + 1
    old_fv4 = ((2 & 0x3) << 2) | (old_reset & 0x3)
    handoff = self.observe_source(target_id=11, angle_raw=100, b26=0x2F, fv4=old_fv4)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    source = self.observe_source(target_id=11, angle_raw=100, b26=0x30, fv4=old_fv4)

    # 0x00F advances first. Safety must still accept the exact source generation
    # (or its bounded ID11 angle substitution) by matching native 0x08A itself.
    self.assertTrue(self.safety.safety_rx_hook(self.sync(new_reset)))
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(100)
    replacement = self.host_frame(source, angle_raw=105, mutate_mac=True)
    self.assertTrue(self.safety.safety_tx_hook(replacement))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

    new_fv4 = ((3 & 0x3) << 2) | (new_reset & 0x3)
    next_source = self.observe_source(target_id=11, angle_raw=105, b26=0x31, fv4=new_fv4)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(next_source, angle_raw=110, mutate_mac=True)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

  def test_modified_id11_preserves_exact_fv4(self):
    handoff = self.observe_source(target_id=11, angle_raw=100, b26=0x11, fv4=8)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))
    source = self.observe_source(target_id=11, angle_raw=100, b26=0x12, fv4=9)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(100)
    data = bytearray(self.host_frame(source, angle_raw=101, mutate_mac=True)[0].data)[:32]
    data[28] ^= 0x10  # change FV4, not merely MAC28
    msg = libsafety_py.make_CANPacket(0x08A, 0, bytes(data))
    msg[0].fd = 1
    self.assertFalse(self.safety.safety_tx_hook(msg))

  def test_recent_native_history_allows_oracle_reply_lag(self):
    handoff = self.observe_source(target_id=11, angle_raw=90, b26=0x1F, semantic=0x6F)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(handoff)))

    source0 = self.observe_source(target_id=11, angle_raw=100, b26=0x20, semantic=0x70)
    # A later native generation can arrive while command5 is signing source0.
    self.observe_source(target_id=18, angle_raw=200, b26=0x21, semantic=0x71)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(90)
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(source0, angle_raw=95, mutate_mac=True)))

  def test_host_request_mode_blocks_legacy_c7_sideband(self):
    self.observe_source(target_id=11)
    self.assertFalse(self.safety.safety_tx_hook(self.c7()))
    self.assertTrue(self.safety.safety_tx_hook(self.admin(1)))
    self.assertFalse(self.safety.safety_tx_hook(self.c7()))

  def test_replacement_requires_fd_and_sidebands_require_classic(self):
    source = self.observe_source(target_id=0)
    self.arm()
    clone = self.host_frame(source)
    classic = libsafety_py.make_CANPacket(0x08A, 0, bytes(clone[0].data)[:32])
    self.assertFalse(self.safety.safety_tx_hook(classic))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

    self.observe_source(target_id=0, b26=0x13)
    self.assertTrue(self.safety.safety_tx_hook(self.admin(1)))
    fd_admin = self.admin(0)
    fd_admin[0].fd = 1
    self.assertFalse(self.safety.safety_tx_hook(fd_admin))
    fd_oracle = self.oracle_fragment(0, 1)
    fd_oracle[0].fd = 1
    self.assertFalse(self.safety.safety_tx_hook(fd_oracle))

  def test_watchdog_fails_open(self):
    source = self.observe_source(target_id=0)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.host_frame(source)))
    self.safety.set_timer(99_999)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)
    self.safety.set_timer(100_001)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

  def test_explicit_release_resumes_stock(self):
    self.observe_source(target_id=0)
    self.arm()
    self.assertTrue(self.safety.safety_tx_hook(self.admin(0)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

  def test_without_host_flag_existing_surface_is_unchanged(self):
    param = EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.F33 | ToyotaSafetyFlags.STOCK_LONGITUDINAL
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()
    self.assertTrue(self.safety.safety_rx_hook(self.source_08a()))
    self.assertFalse(self.safety.safety_tx_hook(self.admin(1)))
    self.assertFalse(self.safety.safety_tx_hook(self.oracle_fragment(0, 1)))
    source = bytes(self.source_08a()[0].data)[:32]
    self.assertFalse(self.safety.safety_tx_hook(self.host_frame(source)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)


if __name__ == "__main__":
  unittest.main()
