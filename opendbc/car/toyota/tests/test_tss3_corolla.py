import math
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, CanData, structs
from opendbc.car.fw_versions import match_fw_to_car_exact
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC, EPS_SCALE, FW_QUERY_CONFIG, TOYOTA_PLATFORM_BY_VEHICLE, ToyotaFlags, ToyotaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py


Ecu = structs.CarParams.Ecu

# Representative bus-1 frames copied from Span's tracked 2025 Corolla drive.
SPAN_FRAMES = {
  0x00F: bytes.fromhex("162d0040d8a0a606"),
  0x025: bytes.fromhex("0ff800005fff0092000000000000000000000000000000000000000091f9fcc0"),
  0x030: bytes.fromhex("0a000000220450b80b002000380006d4020c00000038033b00000000706253c1"),
  0x0AA: bytes.fromhex("1abd1a6f1aba1a6f"),
  0x101: bytes.fromhex("8800003a000000cc"),
  0x116: bytes.fromhex("000200007a353eaa"),
  0x127: bytes.fromhex("001000000738d857"),
  0x176: bytes.fromhex("8800000000000007"),
  0x614: bytes.fromhex("000036300000ef04"),
  0x620: bytes.fromhex("0000000080000000"),
}
COROLLA_LONG = bytes.fromhex("44905f82800040034defa3007eaff080023fff100a8fffe40000000000000000")


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


def fingerprint():
  fp = {i: {} for i in range(8)}
  fp[1] = {address: len(data) for address, data in SPAN_FRAMES.items()}
  return fp


def state_packets(counter=COROLLA_LONG[2]):
  frames = dict(SPAN_FRAMES)
  # These state carriers are part of the common Toyota-B DBC. The retained
  # Span excerpt above is the evidence source for Corolla-specific decoding.
  frames.update({0x3B7: bytes(8), 0x51E: b"\x80" + bytes(7), 0x622: bytes(8)})
  return ([CanData(address, data, 1) for address, data in frames.items()] +
          [CanData(0x160, long_with_counter(COROLLA_LONG, counter), 2)])


def with_toyota_checksum(address: int, data: bytes) -> bytes:
  result = bytearray(data)
  result[-1] = (len(result) + (address & 0xFF) + (address >> 8) + sum(result[:-1])) & 0xFF
  return bytes(result)


def update_state(ci):
  state = None
  for i in range(20):
    state = ci.update([(1_000_000_000 + i * 10_000_000, state_packets(COROLLA_LONG[2] + i))])
  return state


def update_control_state(ci, moving: bool = True, counter_offset: int = 0):
  state = None
  for i in range(20):
    packets = state_packets(COROLLA_LONG[2] + counter_offset + i)
    wheel_speeds = bytes.fromhex("1c001c001c001c00" if moving else "1a6f1a6f1a6f1a6f")
    active = bytearray(SPAN_FRAMES[0x176])
    active[0] |= 0x20
    gas = bytearray(SPAN_FRAMES[0x116])
    gas[1] = 0
    packets = [CanData(msg.address,
                       wheel_speeds if msg.address == 0x0AA else with_toyota_checksum(0x116, gas) if msg.address == 0x116 else
                       with_toyota_checksum(0x176, active) if msg.address == 0x176 else msg.dat,
                       msg.src)
               for msg in packets]
    state = ci.update([(1_000_000_000 + i * 10_000_000, packets)])
  return state


def control(angle, active=True, accel=0.0, long_active=False):
  cc = structs.CarControl()
  cc.enabled = True
  cc.latActive = active
  cc.longActive = long_active
  cc.actuators.steeringAngleDeg = angle
  cc.actuators.accel = accel
  return cc.as_reader()


class TestToyotaCorollaTSS3(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint(), [], True, False, False)

  def test_platform_contract_and_identities(self):
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
    self.assertTrue(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.TSS3_SIGNER)
    self.assertTrue(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.COROLLA_HF)
    self.assertFalse(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL)
    self.assertEqual(DBC[CAR.TOYOTA_COROLLA_TSS3][Bus.pt], "toyota_tss3_pt_generated")

    self.assertEqual(FW_VERSIONS[CAR.TOYOTA_COROLLA_TSS3][(Ecu.eps, 0x7A1, None)], [
      bytes.fromhex("023839363546313230383030300000000038413331313132303230303000000000"),
      bytes.fromhex("023839363546313230383030300000000038413331313132313330303000000000"),
    ])
    for vehicle_type in (12512, 12513, 12514, 12515, 12516, 12821, 12822, 12823, 12824, 12827):
      self.assertEqual(TOYOTA_PLATFORM_BY_VEHICLE[("NA", vehicle_type)], CAR.TOYOTA_COROLLA_TSS3)
    self.assertTrue(any(request.bus == 1 and request.whitelist_ecus == [Ecu.eps, Ecu.abs] and not request.obd_multiplexing
                        for request in FW_QUERY_CONFIG.requests))

  def test_each_retained_eps_identity_exactly_fingerprints_corolla_tss3(self):
    for version in FW_VERSIONS[CAR.TOYOTA_COROLLA_TSS3][(Ecu.eps, 0x7A1, None)]:
      self.assertEqual(match_fw_to_car_exact({(0x7A1, None): {version}}, match_brand="toyota", log=False),
                       {str(CAR.TOYOTA_COROLLA_TSS3)})

  def test_alpha_long_gating(self):
    cp = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint(), [], False, False, False)
    self.assertTrue(cp.alphaLongitudinalAvailable)
    self.assertFalse(cp.openpilotLongitudinalControl)
    self.assertFalse(cp.autoResumeSng)
    self.assertTrue(cp.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL)

  def test_real_span_frames_decode_target_native_state(self):
    ci = CarInterface(self.CP)
    self.assertEqual(ci.can_parsers[Bus.pt].bus, 1)
    state = update_state(ci)
    self.assertTrue(state.canValid)
    self.assertTrue(state.brakePressed)
    self.assertTrue(state.gasPressed)
    self.assertEqual(state.gearShifter, structs.CarState.GearShifter.drive)
    self.assertAlmostEqual(state.steeringAngleDeg, -11.5)
    self.assertAlmostEqual(state.steeringRateDeg, -1.0)
    self.assertAlmostEqual(state.steeringTorque, 1.06, places=6)
    self.assertFalse(state.vehicleSensorsInvalid)
    self.assertFalse(state.cruiseState.enabled)

  def test_receiver_fields_and_controller_sideband(self):
    packer = CANPacker("toyota_tss3_pt_generated")
    parser = CANParser("toyota_tss3_pt_generated", [("TSS3_LATERAL_CONTROL", float("nan"))], 1)
    msg = packer.make_can_msg("TSS3_LATERAL_CONTROL", 1, {
      "TARGET_LATERAL_ID": 11, "TARGET_STEERING_ANGLE": 1024 / 17870, "SEQUENCE": 63,
    })
    parser.update([(1_000_000_000, [msg])])
    self.assertEqual(parser.vl["TSS3_LATERAL_CONTROL"]["TARGET_LATERAL_ID"], 11)
    self.assertEqual(parser.vl["TSS3_LATERAL_CONTROL"]["SEQUENCE"], 63)
    self.assertTrue(math.isclose(parser.vl["TSS3_LATERAL_CONTROL"]["TARGET_STEERING_ANGLE"], 1024 / 17870,
                                 rel_tol=0, abs_tol=1e-6))

    ci = CarInterface(self.CP)
    update_control_state(ci)
    _, sends = ci.apply(control(5.0, accel=-1.3, long_active=True), 2_000_000_000)
    self.assertEqual(len(sends), 2)
    address, data, bus = next(msg for msg in sends if msg[0] == 0x1FDC0002)
    self.assertEqual((address, bus, len(data)), (0x1FDC0002, 1, 8))
    self.assertEqual(data[:4], b"\x00\xC7\x01\x00")
    self.assertEqual(data[6:], b"\x00\x00")
    _, long_data, long_bus = next(msg for msg in sends if msg[0] == 0x160)
    self.assertEqual(long_bus, 0)
    self.assertEqual(long_data[2], (COROLLA_LONG[2] + 19) & 0xFF)
    self.assertEqual(long_data[3], COROLLA_LONG[3])
    self.assertEqual(long_data[4:6], bytes.fromhex("faec"))
    self.assertEqual(long_data[6:], COROLLA_LONG[6:])

    state = update_control_state(ci, moving=False, counter_offset=20)
    self.assertLess(state.vEgo, 0.45)
    output, sends = ci.apply(control(5.0, accel=-1.3, long_active=True), 2_100_000_000)
    _, long_data, _ = next(msg for msg in sends if msg[0] == 0x160)
    self.assertEqual(long_data, long_with_counter(COROLLA_LONG, COROLLA_LONG[2] + 39))
    self.assertEqual(output.accel, 0.0)


class TestToyotaCorollaTSS3Safety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    param = (EPS_SCALE[CAR.TOYOTA_COROLLA_TSS3] |
             ToyotaSafetyFlags.TSS3_SIGNER | ToyotaSafetyFlags.COROLLA_HF)
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()
    active = bytearray(SPAN_FRAMES[0x176])
    active[0] |= 0x20
    neutral = dict(SPAN_FRAMES)
    neutral[0x116] = bytes(8)
    neutral[0x101] = bytes(8)
    for address in (0x025, 0x030, 0x0AA, 0x116, 0x101):
      self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, 1, neutral[address])))
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x176, 1, bytes(active))))
    self.assertTrue(self.safety.get_controls_allowed())

  @staticmethod
  def c7(angle_raw=0, sequence=1, bus=1):
    data = b"\x00\xC7" + bytes((sequence, 0)) + angle_raw.to_bytes(2, "big", signed=True) + b"\x00\x00"
    return libsafety_py.make_CANPacket(0x1FDC0002, bus, data)

  def test_only_bounded_c7_on_stock_toyota_b(self):
    self.assertTrue(self.safety.safety_tx_hook(self.c7()))
    self.assertFalse(self.safety.safety_tx_hook(self.c7(bus=0)))
    self.assertFalse(self.safety.safety_tx_hook(self.c7(1746)))
    for index in (0, 1, 3, 6, 7):
      data = bytearray(self.c7()[0].data)
      data[index] ^= 1
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x1FDC0002, 1, bytes(data))))

  def test_native_longitudinal_replacement_is_enabled(self):
    data = long_with_counter(COROLLA_LONG, COROLLA_LONG[2])
    self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x160, 0, data)))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x160, 1, data)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x160), -1)

    gas = bytearray(SPAN_FRAMES[0x116])
    gas[1] = 1
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x116, 1, bytes(gas))))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x160), 0)

  def test_native_cruise_gate_revokes_control(self):
    disabled = bytearray(SPAN_FRAMES[0x176])
    disabled[0] &= ~0x20
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x176, 1, bytes(disabled))))
    self.assertFalse(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
