import unittest

from opendbc.can import CANParser
from opendbc.car import Bus, CanData, structs
from opendbc.car.fw_versions import match_fw_to_car_exact
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.toyotacan import toyota_e2e_p05_checksum
from opendbc.car.toyota.values import CAR, DBC, CarControllerParams, ToyotaFlags, ToyotaSafetyFlags


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


def control(angle: float, active: bool = True, accel: float = 0.0, long_active: bool = False, enabled: bool = True,
            cancel: bool = False, left_lane: bool = False, right_lane: bool = False, steer_alert: bool = False,
            lead_distance_bars: int = 0):
  cc = structs.CarControl()
  cc.enabled = enabled
  cc.latActive = enabled and active
  cc.longActive = enabled and long_active
  cc.cruiseControl.cancel = cancel
  cc.actuators.steeringAngleDeg = angle
  cc.actuators.accel = accel
  cc.hudControl.leftLaneVisible = left_lane
  cc.hudControl.rightLaneVisible = right_lane
  cc.hudControl.leadDistanceBars = lead_distance_bars
  if steer_alert:
    cc.hudControl.visualAlert = structs.CarControl.HUDControl.VisualAlert.steerRequired
  return cc.as_reader()


class TestToyotaCamryTSS3(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], True, False, False)

  def test_platform_contract(self):
    self.assertTrue(self.CP.flags & ToyotaFlags.TSS3)
    self.assertFalse(self.CP.flags & ToyotaFlags.SECOC)
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
    self.assertFalse(relay.alphaLongitudinalAvailable)
    self.assertTrue(relay.openpilotLongitudinalControl)
    self.assertTrue(relay.autoResumeSng)
    self.assertTrue(relay.pcmCruise)
    self.assertAlmostEqual(relay.longitudinalActuatorDelay, 0.2)
    self.assertEqual(list(relay.longitudinalTuning.kiBP), [0.])
    self.assertEqual(list(relay.longitudinalTuning.kiV), [0.])
    self.assertFalse(relay.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL.value)

    relay_without_toggle = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], False, False, False)
    self.assertFalse(relay_without_toggle.alphaLongitudinalAvailable)
    self.assertTrue(relay_without_toggle.openpilotLongitudinalControl)
    self.assertTrue(relay_without_toggle.pcmCruise)
    self.assertFalse(relay_without_toggle.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL.value)

  def test_unqualified_camry_longitudinal_is_not_advertised(self):
    for alpha_long in (False, True):
      cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], alpha_long, False, False)
      self.assertFalse(cp.alphaLongitudinalAvailable)
      self.assertFalse(cp.openpilotLongitudinalControl)
      self.assertFalse(cp.autoResumeSng)
      self.assertTrue(cp.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL)

  def test_exact_identity(self):
    fw = FW_VERSIONS[CAR.TOYOTA_CAMRY_TSS3]
    self.assertEqual(fw[(Ecu.eps, 0x7A1, None)], [
      bytes.fromhex("023839363546333330373030300000000038413331313333303331303000000000")])

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
    max_delta = CarControllerParams.F33_ANGLE_LIMITS.MAX_ANGLE_RATE * ci.CC.params.STEER_STEP
    output, sends = ci.apply(control(0.0, active=False, enabled=False), 2_000_000_000)
    self.assertAlmostEqual(output.steeringAngleDeg, measured, delta=0.01)

    # Engagement between source publications holds the ordinary inactive
    # baseline. It is acquisition latency, not a steering/ACC fault.
    output, sends = ci.apply(control(5.0), 2_010_000_000)
    self.assertFalse(any(address == 0x777 and data[1] == 0xC7 for address, data, _ in sends))
    self.assertAlmostEqual(output.steeringAngleDeg, measured, delta=0.01)

    # The next native publication creates exactly one normally rate-limited
    # actuator application. Toyota's source ID is not a controller veto.
    request[21] = request[21] & 0xC0
    update_state(ci, moving=True, control_request=bytes(request), bus=0, source_bus=2, hud=CAMRY_HUD)
    output, sends = ci.apply(control(20.0), 2_020_000_000)
    self.assertFalse(any(address == 0x777 and data[1] == 0xC7 for address, data, _ in sends))
    self.assertGreater(output.steeringAngleDeg, measured)
    self.assertLessEqual(output.steeringAngleDeg, measured + max_delta)

    # No new application generation means no hidden 100 Hz actuator advance.
    held_angle = output.steeringAngleDeg
    output, _ = ci.apply(control(20.0), 2_030_000_000)
    self.assertAlmostEqual(output.steeringAngleDeg, held_angle, delta=0.001)

  def test_host_request_plane_exposes_bounded_alpha_long_acceleration(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)
    ci = CarInterface(cp)
    update_state(ci, moving=True, bus=0, source_bus=2, hud=CAMRY_HUD)

    for requested, expected in ((1.2, 1.2), (2.0, 2.0), (-2.0, -2.0), (3.0, 2.0), (-4.0, -3.5)):
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

  def test_lta_uses_normal_button_events(self):
    ci = CarInterface(self.CP)
    state = update_state(ci, hud=bytes.fromhex("1200002202ee9307"))
    self.assertEqual(list(state.buttonEvents), [])

    distance = bytearray(CAMRY_COMMON[0x251])
    distance[5] = (distance[5] & 0x1F) | (2 << 5)
    state = update_state(ci, counter_offset=20, hud=bytes.fromhex("1200002202ee9307"),
                         cruise_display=bytes(distance), iterations=1)
    self.assertEqual(list(state.buttonEvents), [])

    state = update_state(ci, counter_offset=21, hud=bytes.fromhex("1000002200ee9307"),
                         cruise_display=bytes(distance), iterations=1)
    self.assertEqual([(event.type, event.pressed) for event in state.buttonEvents], [
      (structs.CarState.ButtonEvent.Type.lkas, True),
      (structs.CarState.ButtonEvent.Type.lkas, False),
    ])

  def test_relay_gap_policy_maps_four_absolute_positions_to_three_personalities(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)
    ci = CarInterface(cp)
    distance = bytearray(CAMRY_COMMON[0x251])

    def set_distance(state: int, counter: int):
      distance[5] = (distance[5] & 0x1F) | (state << 5)
      return update_state(ci, bus=0, source_bus=2, counter_offset=counter,
                          cruise_display=bytes(distance), hud=CAMRY_HUD, iterations=1)

    def gap_events(state):
      return [(event.type, event.pressed) for event in state.buttonEvents
              if event.type == structs.CarState.ButtonEvent.Type.gapAdjustCruise]

    # Position 1 maps to relaxed. Starting from aggressive requires one normal
    # decrement/wrap event, then waits for standard CarControl feedback.
    ci.apply(control(0.0, lead_distance_bars=1), 2_000_000_000)
    self.assertEqual(gap_events(set_distance(1, 20)), [
      (structs.CarState.ButtonEvent.Type.gapAdjustCruise, True),
      (structs.CarState.ButtonEvent.Type.gapAdjustCruise, False),
    ])
    self.assertEqual(gap_events(set_distance(1, 21)), [])
    ci.apply(control(0.0, lead_distance_bars=3), 2_010_000_000)
    self.assertEqual(gap_events(set_distance(1, 22)), [])

    # Positions 2 and 3 both map to standard, so the middle Toyota transition
    # cannot advance openpilot and create a four-state/three-state phase drift.
    self.assertEqual(len(gap_events(set_distance(2, 23))), 2)
    ci.apply(control(0.0, lead_distance_bars=2), 2_020_000_000)
    self.assertEqual(gap_events(set_distance(3, 24)), [])

    self.assertEqual(len(gap_events(set_distance(4, 25))), 2)
    ci.apply(control(0.0, lead_distance_bars=1), 2_030_000_000)
    self.assertEqual(len(gap_events(set_distance(1, 26))), 2)

  def test_relay_gap_policy_converges_a_two_step_absolute_correction(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)
    ci = CarInterface(cp)
    distance = bytearray(CAMRY_COMMON[0x251])
    distance[5] = (distance[5] & 0x1F) | (4 << 5)  # aggressive

    ci.apply(control(0.0, lead_distance_bars=3), 2_000_000_000)  # relaxed
    first = update_state(ci, bus=0, source_bus=2, counter_offset=20,
                         cruise_display=bytes(distance), hud=CAMRY_HUD, iterations=1)
    self.assertEqual(len(first.buttonEvents), 2)
    # Do not emit the second step until the first personality change appears in
    # the ordinary leadDistanceBars feedback.
    waiting = update_state(ci, bus=0, source_bus=2, counter_offset=21,
                           cruise_display=bytes(distance), hud=CAMRY_HUD, iterations=1)
    self.assertEqual(list(waiting.buttonEvents), [])
    ci.apply(control(0.0, lead_distance_bars=2), 2_010_000_000)  # standard
    second = update_state(ci, bus=0, source_bus=2, counter_offset=22,
                          cruise_display=bytes(distance), hud=CAMRY_HUD, iterations=1)
    self.assertEqual(len(second.buttonEvents), 2)
    ci.apply(control(0.0, lead_distance_bars=1), 2_020_000_000)  # aggressive
    settled = update_state(ci, bus=0, source_bus=2, counter_offset=23,
                           cruise_display=bytes(distance), hud=CAMRY_HUD, iterations=1)
    self.assertEqual(list(settled.buttonEvents), [])

  def test_reengagement_does_not_reuse_the_residents_consumed_sequence(self):
    ci = CarInterface(self.CP)
    update_state(ci)
    frames = []
    for active in (True, False, True):
      _, sends = ci.apply(control(1.0, active=active), 2_000_000_000)
      frames.append(next(data for address, data, _ in sends if address == 0x777))
      ci.apply(control(1.0, active=active), 2_010_000_000)
    self.assertEqual([frame[3] for frame in frames], [1, 0, 3])

if __name__ == "__main__":
  unittest.main()
