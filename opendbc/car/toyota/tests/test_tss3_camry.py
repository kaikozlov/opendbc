import unittest

from opendbc.car import Bus, CanData, structs
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.fw_query_definitions import PlatformResolverContext
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC, EPS_SCALE, ToyotaFlags, ToyotaSafetyFlags, resolve_platform
from opendbc.car.vin import VIN_UNKNOWN
from opendbc.safety.tests.libsafety import libsafety_py


Ecu = structs.CarParams.Ecu

CAMRY_COMMON = {
  0x025: bytes.fromhex("000100005000007e0000000000000000000000000000000000000000bb6fee54"),
  0x030: bytes.fromhex("00000000170000500000100026820000000000010000ffff00000000b280595f"),
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


def update_state(ci: CarInterface, moving: bool = False, counter_offset: int = 0):
  state = None
  for i in range(20):
    frames = dict(CAMRY_COMMON)
    if moving:
      frames[0x0AA] = bytes.fromhex("1c001c001c001c00")
    packets = ([CanData(address, data, 1) for address, data in frames.items()] +
               [CanData(0x160, long_with_counter(CAMRY_LONG, CAMRY_LONG[2] + counter_offset + i), 2)])
    state = ci.update([(1_000_000_000 + i * 10_000_000, packets)])
  return state


def control(angle: float, active: bool = True, accel: float = 0.0, long_active: bool = False):
  cc = structs.CarControl()
  cc.enabled = True
  cc.latActive = active
  cc.longActive = long_active
  cc.actuators.steeringAngleDeg = angle
  cc.actuators.accel = accel
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
    self.assertTrue(self.CP.autoResumeSng)
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

  def test_exact_abs_identity_resolves_with_dead_eps_diagnostics(self):
    version = FW_VERSIONS[CAR.TOYOTA_CAMRY_TSS3][(Ecu.abs, 0x7B0, None)][0]
    car_fw = [structs.CarParams.CarFw(ecu=Ecu.abs, fwVersion=version, brand="toyota", address=0x7B0)]
    exact, matches = match_fw_to_car(car_fw, VIN_UNKNOWN, allow_fuzzy=False)
    self.assertTrue(exact)
    self.assertEqual(matches, {str(CAR.TOYOTA_CAMRY_TSS3)})

    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), car_fw, False, False, False)
    self.assertTrue(cp.flags & ToyotaFlags.EPS_DIAGNOSTICS_UNAVAILABLE)
    self.assertFalse(cp.openpilotLongitudinalControl)
    self.assertEqual(cp.minSteerSpeed, 1000.)
    self.assertFalse(cp.steerAtStandstill)
    parsers = CarState.get_can_parsers(cp)
    self.assertTrue(parsers[Bus.pt].message_states[0x030].ignore_alive)

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
    return libsafety_py.make_CANPacket(0x160, bus, bytes(data))

  def test_tss3_longitudinal_bounds_and_dynamic_forwarding(self):
    self.assertTrue(self.safety.safety_tx_hook(self.long_request(2.0)))
    self.assertTrue(self.safety.safety_tx_hook(self.long_request(-3.5)))
    self.assertFalse(self.safety.safety_tx_hook(self.long_request(2.1)))
    self.assertFalse(self.safety.safety_tx_hook(self.long_request(-3.6)))
    self.assertFalse(self.safety.safety_tx_hook(self.long_request(0.0, bus=1)))
    unsafe_coarse = bytearray(self.long_request(0.0).data)
    unsafe_coarse[12] = (-30) & 0x7F
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


if __name__ == "__main__":
  unittest.main()
