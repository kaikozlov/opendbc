import math
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, CanData, structs
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


def fingerprint():
  fp = {i: {} for i in range(8)}
  fp[1] = {address: len(data) for address, data in SPAN_FRAMES.items()}
  return fp


def state_packets():
  frames = dict(SPAN_FRAMES)
  # These state carriers are part of the common Toyota-B DBC. The retained
  # Span excerpt above is the evidence source for Corolla-specific decoding.
  frames.update({0x3B7: bytes(8), 0x51E: b"\x80" + bytes(7), 0x622: bytes(8)})
  return [CanData(address, data, 1) for address, data in frames.items()]


def update_state(ci):
  state = None
  for i in range(20):
    state = ci.update([(1_000_000_000 + i * 10_000_000, state_packets())])
  return state


def control(angle, active=True):
  cc = structs.CarControl()
  cc.enabled = True
  cc.latActive = active
  cc.actuators.steeringAngleDeg = angle
  return cc.as_reader()


class TestToyotaCorollaTSS3(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint(), [], False, False, False)

  def test_platform_contract_and_identities(self):
    self.assertTrue(self.CP.flags & ToyotaFlags.TSS3)
    self.assertTrue(self.CP.flags & ToyotaFlags.SECOC)
    self.assertFalse(self.CP.flags & ToyotaFlags.TSS2)
    self.assertFalse(self.CP.dashcamOnly)
    self.assertFalse(self.CP.secOcRequired)
    self.assertFalse(self.CP.openpilotLongitudinalControl)
    self.assertEqual(self.CP.steerControlType, structs.CarParams.SteerControlType.angle)
    self.assertEqual(self.CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.toyota)
    self.assertTrue(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.TSS3_SIGNER)
    self.assertTrue(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.COROLLA_HF)
    self.assertTrue(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL)
    self.assertEqual(DBC[CAR.TOYOTA_COROLLA_TSS3][Bus.pt], "toyota_tss3_pt_generated")

    self.assertEqual(FW_VERSIONS[CAR.TOYOTA_COROLLA_TSS3][(Ecu.eps, 0x7A1, None)], [
      bytes.fromhex("023839363546313230383030300000000038413331313132303230303000000000"),
      bytes.fromhex("023839363546313230383030300000000038413331313132313330303000000000"),
    ])
    for vehicle_type in (12512, 12513, 12514, 12515, 12516, 12821, 12822, 12823, 12824, 12827):
      self.assertEqual(TOYOTA_PLATFORM_BY_VEHICLE[("NA", vehicle_type)], CAR.TOYOTA_COROLLA_TSS3)
    self.assertTrue(any(request.bus == 1 and request.whitelist_ecus == [Ecu.eps] and not request.obd_multiplexing
                        for request in FW_QUERY_CONFIG.requests))

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
    update_state(ci)
    _, sends = ci.apply(control(5.0), 2_000_000_000)
    self.assertEqual(len(sends), 1)
    address, data, bus = sends[0]
    self.assertEqual((address, bus, len(data)), (0x1FDC0002, 1, 8))
    self.assertEqual(data[:4], b"\x00\xC7\x01\x00")
    self.assertEqual(data[6:], b"\x00\x00")


class TestToyotaCorollaTSS3Safety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    param = (EPS_SCALE[CAR.TOYOTA_COROLLA_TSS3] | ToyotaSafetyFlags.STOCK_LONGITUDINAL |
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

  def test_native_cruise_gate_revokes_control(self):
    disabled = bytearray(SPAN_FRAMES[0x176])
    disabled[0] &= ~0x20
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x176, 1, bytes(disabled))))
    self.assertFalse(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
