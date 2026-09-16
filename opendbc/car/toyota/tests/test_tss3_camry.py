import unittest

from opendbc.car import Bus, CanData, structs
from opendbc.car.fw_versions import match_fw_to_car_exact
from opendbc.car.fw_query_definitions import PlatformResolverContext
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.toyotacan import toyota_e2e_p05_checksum
from opendbc.car.toyota.values import CAR, DBC, EPS_SCALE, ToyotaFlags, ToyotaSafetyFlags, resolve_platform
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


def long_with_counter(template: bytes, counter: int) -> bytes:
  data = bytearray(template)
  data[2] = counter & 0xFF
  crc = 0
  for byte in (*data[2:], 0x4A, 0x44):
    crc ^= byte << 8
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
  data[:2] = crc.to_bytes(2, "little")
  return bytes(data)


def fingerprint() -> dict[int, dict[int, int]]:
  fp = {i: {} for i in range(8)}
  fp[1] = {address: len(data) for address, data in CAMRY_COMMON.items()}
  return fp


def update_state(ci: CarInterface, moving: bool = False, counter_offset: int = 0, hud: bytes | None = None,
                 eps_status: int | None = None, eps_telemetry: bytes | None = None):
  state = None
  for i in range(20):
    frames = dict(CAMRY_COMMON)
    if eps_telemetry is not None:
      frames[0x030] = eps_telemetry
    if eps_status is not None:
      eps = bytearray(frames[0x030])
      eps[6] = eps_status
      eps[7] = (sum(eps[:7]) + 0x38) & 0xFF
      frames[0x030] = bytes(eps)
    if moving:
      frames[0x0AA] = bytes.fromhex("1c001c001c001c00")
    packets = ([CanData(address, data, 1) for address, data in frames.items()] +
               [CanData(0x160, long_with_counter(CAMRY_LONG, CAMRY_LONG[2] + counter_offset + i), 2)])
    if hud is not None:
      packets.append(CanData(0x412, hud, 1))
    state = ci.update([(1_000_000_000 + i * 10_000_000, packets)])
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
    self.assertTrue(self.CP.openpilotLongitudinalControl)
    self.assertTrue(self.CP.alphaLongitudinalAvailable)
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
    self.assertFalse(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL)
    self.assertEqual(DBC[CAR.TOYOTA_CAMRY_TSS3][Bus.pt], "toyota_tss3_pt_generated")
    self.assertTrue(self.CP.enableBsm)

  def test_alpha_long_gating(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], False, False, False)
    self.assertTrue(cp.alphaLongitudinalAvailable)
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

  def test_exact_eps_identity_fingerprints_camry_tss3(self):
    for version in FW_VERSIONS[CAR.TOYOTA_CAMRY_TSS3][(Ecu.eps, 0x7A1, None)]:
      self.assertEqual(match_fw_to_car_exact({(0x7A1, None): {version}}, match_brand="toyota", log=False),
                       {str(CAR.TOYOTA_CAMRY_TSS3)})

    # Production control is not downgraded based on a transient diagnostic
    # response failure. The exact EPS identity is part of fingerprinting, while
    # runtime capability comes from source-real CAN state.
    self.assertEqual(self.CP.minSteerSpeed, 0.)
    self.assertTrue(self.CP.steerAtStandstill)

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

  def test_carstate_cooperative_inhibits_assert_and_recover(self):
    for command_inhibit, angle_inhibit in ((1, 0), (0, 1), (1, 1)):
      with self.subTest(command=command_inhibit, angle=angle_inhibit):
        ci = CarInterface(self.CP)
        raw = bytearray(CAMRY_COMMON[0x030])
        raw[16] = (raw[16] & ~1) | command_inhibit
        raw[19] = (raw[19] & ~1) | angle_inhibit
        state = update_state(ci, eps_telemetry=bytes(raw), hud=CAMRY_HUD)
        self.assertTrue(state.canValid)
        self.assertTrue(state.steerFaultTemporary)
        self.assertFalse(state.steerFaultPermanent)
        self.assertFalse(state.vehicleSensorsInvalid)
        state = update_state(ci, counter_offset=20, hud=CAMRY_HUD)
        self.assertFalse(state.steerFaultTemporary)
        self.assertFalse(state.steerFaultPermanent)

  def test_reference_initializing_source_is_not_healthy_steering(self):
    # The original fixture is valid telemetry, but F33's reference-inhibit
    # signal at B19[0] is set. Do not silently label this startup state healthy.
    initializing = bytes.fromhex("00000000170000500000100026820000000000010000ffff00000000b280595f")
    state = update_state(CarInterface(self.CP), eps_telemetry=initializing, hud=CAMRY_HUD)
    self.assertTrue(state.canValid)
    self.assertTrue(state.steerFaultTemporary)
    self.assertFalse(state.steerFaultPermanent)
    self.assertFalse(state.vehicleSensorsInvalid)

  def test_unrelated_eps_status_bits_are_not_promoted_to_faults(self):
    for status in (0, 1, 2, 8, 0xF0):
      with self.subTest(status=status):
        state = update_state(CarInterface(self.CP), eps_status=status)
        self.assertFalse(state.steerFaultTemporary)
        self.assertFalse(state.steerFaultPermanent)
        self.assertEqual(state.vehicleSensorsInvalid, bool(status & 1))

  def test_controller_emits_c7_and_template_preserving_longitudinal_request(self):
    ci = CarInterface(self.CP)
    update_state(ci, moving=True)
    output, sends = ci.apply(control(5.0, accel=1.2, long_active=True), 2_000_000_000)
    self.assertEqual(len(sends), 2)
    address, data, bus = next(msg for msg in sends if msg[0] == 0x1FDC0002)
    self.assertEqual((address, bus, len(data)), (0x1FDC0002, 1, 8))
    self.assertEqual(data[:4], b"\x00\xC7\x01\x00")
    self.assertEqual(data[6:], b"\x00\x00")
    self.assertAlmostEqual(output.steeringAngleDeg,
                           int.from_bytes(data[4:6], "big", signed=True) * (1024 / 17870), delta=0.03)
    _, long_data, long_bus = next(msg for msg in sends if msg[0] == 0x160)
    self.assertEqual(long_bus, 0)
    expected = bytearray(CAMRY_LONG)
    expected[4:6] = bytes.fromhex("84b0")
    expected[12] = 0x74
    self.assertEqual(long_data, long_with_counter(expected, CAMRY_LONG[2] + 19))

    state = update_state(ci, counter_offset=20)
    self.assertLess(state.vEgo, 0.45)
    output, sends = ci.apply(control(5.0, accel=1.2, long_active=True), 2_100_000_000)
    _, long_data, _ = next(msg for msg in sends if msg[0] == 0x160)
    self.assertEqual(long_data, long_with_counter(CAMRY_LONG, CAMRY_LONG[2] + 39))
    self.assertEqual(output.accel, 0.0)

  def test_camry_coarse_request_uses_bounded_inverse_tenths(self):
    ci = CarInterface(self.CP)
    update_state(ci, moving=True)
    output, sends = ci.apply(control(0.0, accel=1.5, long_active=True), 2_000_000_000)
    _, data, _ = next(msg for msg in sends if msg[0] == 0x160)
    self.assertAlmostEqual(output.accel, 1.3)
    self.assertEqual(data[4:6], bytes.fromhex("8514"))
    self.assertEqual(data[12], 0x73)

  def test_inactive_c7_tracks_measured_angle_with_neutral_sequence(self):
    ci = CarInterface(self.CP)
    state = update_state(ci)
    _, sends = ci.apply(control(20.0, False), 2_000_000_000)
    _, data, bus = next(msg for msg in sends if msg[0] == 0x1FDC0002)
    self.assertEqual(bus, 1)
    self.assertEqual(data[:4], b"\x00\xC7\x00\x00")
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

  def test_reengagement_does_not_reuse_the_residents_consumed_sequence(self):
    ci = CarInterface(self.CP)
    update_state(ci)
    frames = []
    for active in (True, False, True):
      _, sends = ci.apply(control(1.0, active=active), 2_000_000_000)
      frames.append(next(data for address, data, _ in sends if address == 0x1FDC0002))
      ci.apply(control(1.0, active=active), 2_010_000_000)
    self.assertEqual([frame[2] for frame in frames], [1, 0, 2])


class TestToyotaCamryTSS3Safety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    param = EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.F33
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()
    for address in (0x025, 0x0AA, 0x116, 0x101, 0x08A):
      self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, 1, CAMRY_COMMON[address])))
    self.assertTrue(self.safety.get_controls_allowed())

  @staticmethod
  def c7(angle_raw: int = 0, sequence: int = 1, bus: int = 1):
    data = b"\x00\xC7" + bytes((sequence, 0)) + angle_raw.to_bytes(2, "big", signed=True) + b"\x00\x00"
    return libsafety_py.make_CANPacket(0x1FDC0002, bus, data)

  def test_accepts_bounded_c7_only_on_unsplit_bus(self):
    self.assertTrue(self.safety.safety_tx_hook(self.c7()))
    self.assertFalse(self.safety.safety_tx_hook(self.c7(bus=0)))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x08A, 1, CAMRY_COMMON[0x08A])))

  @staticmethod
  def long_request(accel: float, bus: int = 0):
    data = bytearray(CAMRY_LONG)
    raw = round(accel / 0.001) & 0x7FFF
    data[4] = (data[4] & 0x80) | (raw >> 8)
    data[5] = raw & 0xFF
    data[12] = round(-accel / 0.1) & 0x7F
    data[:2] = toyota_e2e_p05_checksum(0x160, data).to_bytes(2, "little")
    return libsafety_py.make_CANPacket(0x160, bus, bytes(data))

  def test_tss3_longitudinal_bounds_and_dynamic_forwarding(self):
    self.assertTrue(self.safety.safety_tx_hook(self.long_request(1.3)))
    self.assertTrue(self.safety.safety_tx_hook(self.long_request(-1.5)))
    self.assertFalse(self.safety.safety_tx_hook(self.long_request(1.4)))
    self.assertFalse(self.safety.safety_tx_hook(self.long_request(-1.6)))
    self.assertFalse(self.safety.safety_tx_hook(self.long_request(0.0, bus=1)))
    unsafe_coarse = bytearray(self.long_request(0.0).data[0:32])
    unsafe_coarse[12] = (-30) & 0x7F
    unsafe_coarse[:2] = toyota_e2e_p05_checksum(0x160, unsafe_coarse).to_bytes(2, "little")
    self.assertEqual(len(unsafe_coarse), 32)
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x160, 0, bytes(unsafe_coarse))))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x160), -1)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x230), 0)
    gas = bytearray(CAMRY_COMMON[0x116])
    gas[1] = 1
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x116, 1, bytes(gas))))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x160), 0)

  def test_rejects_bad_header_reserved_bytes_and_overangle(self):
    for index in (0, 1, 3, 6, 7):
      data = bytearray(self.c7()[0].data)
      data[index] ^= 1
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x1FDC0002, 1, bytes(data))))

    self.assertFalse(self.safety.safety_tx_hook(self.c7(1746)))

  def test_stock_cruise_latch_owns_controls_allowed(self):
    disabled = bytearray(CAMRY_COMMON[0x08A])
    disabled[3] &= ~0x08
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 1, bytes(disabled))))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(self.c7()))

  def test_byte_exact_stock_handoff_is_not_clipped_to_host_limits(self):
    # Native August-27 drive A, segment 4, 308202253232 ns: the measured
    # B4:B5 quantity is -1.532, outside the bounded host request envelope.
    stock = bytes.fromhex("8e78bf82fa04400537281b000da81280022f80c0000fffc40000000000000000")
    tx = libsafety_py.make_CANPacket(0x160, 0, stock)
    self.assertFalse(self.safety.safety_tx_hook(tx))
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x160, 2, stock)))
    self.assertTrue(self.safety.safety_tx_hook(tx))
    changed = bytearray(stock)
    changed[13] ^= 1
    changed[:2] = toyota_e2e_p05_checksum(0x160, changed).to_bytes(2, "little")
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x160, 0, changed)))
    corrupt = bytearray(stock)
    corrupt[0] ^= 1
    self.assertFalse(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x160, 2, corrupt)))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x160, 0, corrupt)))

  def test_corrupt_host_longitudinal_packet_is_rejected(self):
    packet = self.long_request(0.5)
    packet[0].data[0] ^= 1
    self.assertFalse(self.safety.safety_tx_hook(packet))

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


if __name__ == "__main__":
  unittest.main()
