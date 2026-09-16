import math
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, CanData, structs
from opendbc.car.fw_versions import match_fw_to_car_exact
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC, EPS_SCALE, FW_QUERY_CONFIG, TOYOTA_COROLLA_TSS3_HYBRID_VEHICLE_TYPES, \
                                      TOYOTA_COROLLA_TSS3_ICE_VEHICLE_TYPES, TOYOTA_PLATFORM_BY_VEHICLE, ToyotaFlags, ToyotaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py


Ecu = structs.CarParams.Ecu

# Representative bus-1 frames copied from Span's tracked 2025 Corolla drive.
SPAN_FRAMES = {
  0x00F: bytes.fromhex("162d0040d8a0a606"),
  0x025: bytes.fromhex("0ff800005fff0092000000000000000000000000000000000000000091f9fcc0"),
  0x030: bytes.fromhex("0a000000220450b80b002000380006d4020c00000038033b00000000706253c1"),
  0x08A: bytes.fromhex("000000008000001203260003267fff007fffff3600000000000039009064215b"),
  0x0AA: bytes.fromhex("1abd1a6f1aba1a6f"),
  0x101: bytes.fromhex("8800003a000000cc"),
  0x116: bytes.fromhex("000200007a353eaa"),
  0x127: bytes.fromhex("001000000738d857"),
  0x176: bytes.fromhex("8800000000000007"),
  0x251: bytes.fromhex("a00000488068a080"),
  0x614: bytes.fromhex("000036300000ef04"),
  0x620: bytes.fromhex("0000000080000000"),
}
COROLLA_LONG = bytes.fromhex("44905f82800040034defa3007eaff080023fff100a8fffe40000000000000000")
SPAN_ACC_ACTIVE = bytes.fromhex("000000008004475d00520000527fff007fff00390000100000003000749773c5")
# Direct 0x3BF frames from Albino's retained 2023 public Corolla route. The route
# transitions P -> R -> D while these one-hot values change 0x80 -> 0x40 -> 0x10.
ALBINO_GEAR = {
  "P": bytes.fromhex("8000010074d0de47"),
  "R": bytes.fromhex("400001006f306582"),
  "D": bytes.fromhex("100001005fc18f5f"),
}


def fingerprint(hybrid: bool = True):
  fp = {i: {} for i in range(8)}
  frames = dict(SPAN_FRAMES)
  if not hybrid:
    frames.pop(0x127)
    frames[0x3BF] = ALBINO_GEAR["D"]
  fp[1] = {address: len(data) for address, data in frames.items()}
  return fp


def state_packets(*, hybrid: bool = True, gear: bytes | None = None):
  frames = dict(SPAN_FRAMES)
  if hybrid:
    if gear is not None:
      frames[0x127] = gear
  else:
    frames.pop(0x127)
    frames[0x3BF] = ALBINO_GEAR["D"] if gear is None else gear
  # These state carriers are part of the common Toyota-B DBC. The retained
  # Span excerpt above is the evidence source for Corolla-specific decoding.
  frames.update({0x3B7: bytes(8), 0x51E: b"\x80" + bytes(7), 0x622: bytes(8)})
  return [CanData(address, data, 1) for address, data in frames.items()]


def with_toyota_checksum(address: int, data: bytes) -> bytes:
  result = bytearray(data)
  result[-1] = (len(result) + (address & 0xFF) + (address >> 8) + sum(result[:-1])) & 0xFF
  return bytes(result)


def update_state(ci):
  state = None
  for i in range(20):
    state = ci.update([(1_000_000_000 + i * 10_000_000, state_packets())])
  return state


def update_control_state(ci, moving: bool = True, counter_offset: int = 0, *,
                         acc_frame: bytes = SPAN_ACC_ACTIVE, set_speed_mph: int = 0):
  state = None
  display = bytearray(SPAN_FRAMES[0x251])
  display[2] = set_speed_mph
  for i in range(20):
    packets = state_packets()
    wheel_speeds = bytes.fromhex("1c001c001c001c00" if moving else "1a6f1a6f1a6f1a6f")
    gas = bytearray(SPAN_FRAMES[0x116])
    gas[1] = 0
    packets = [CanData(msg.address,
                       wheel_speeds if msg.address == 0x0AA else with_toyota_checksum(0x116, gas) if msg.address == 0x116 else
                       acc_frame if msg.address == 0x08A else bytes(display) if msg.address == 0x251 else msg.dat,
                       msg.src)
               for msg in packets]
    state = ci.update([(1_000_000_000 + (counter_offset + i) * 10_000_000, packets)])
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
    self.assertTrue(self.CP.flags & ToyotaFlags.HYBRID)
    self.assertFalse(self.CP.flags & ToyotaFlags.TSS2)
    self.assertAlmostEqual(self.CP.wheelbase, 2.70)
    self.assertFalse(self.CP.dashcamOnly)
    self.assertFalse(self.CP.secOcRequired)
    self.assertFalse(self.CP.openpilotLongitudinalControl)
    self.assertFalse(self.CP.alphaLongitudinalAvailable)
    self.assertFalse(self.CP.autoResumeSng)
    self.assertAlmostEqual(self.CP.minEnableSpeed, 19 * 0.44704, places=6)
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
    self.assertEqual(TOYOTA_COROLLA_TSS3_ICE_VEHICLE_TYPES, {12512, 12513, 12516, 12821, 12822, 12827})
    self.assertEqual(TOYOTA_COROLLA_TSS3_HYBRID_VEHICLE_TYPES, {12514, 12515, 12823, 12824})
    for vehicle_type in TOYOTA_COROLLA_TSS3_ICE_VEHICLE_TYPES | TOYOTA_COROLLA_TSS3_HYBRID_VEHICLE_TYPES:
      self.assertEqual(TOYOTA_PLATFORM_BY_VEHICLE[("NA", vehicle_type)], CAR.TOYOTA_COROLLA_TSS3)
    self.assertTrue(any(request.bus == 1 and request.whitelist_ecus == [Ecu.eps, Ecu.abs] and not request.obd_multiplexing
                        for request in FW_QUERY_CONFIG.requests))

  def test_each_retained_eps_identity_exactly_fingerprints_corolla_tss3(self):
    for version in FW_VERSIONS[CAR.TOYOTA_COROLLA_TSS3][(Ecu.eps, 0x7A1, None)]:
      self.assertEqual(match_fw_to_car_exact({(0x7A1, None): {version}}, match_brand="toyota", log=False),
                       {str(CAR.TOYOTA_COROLLA_TSS3)})

  def test_alpha_long_cannot_enable_without_08a_sender(self):
    for alpha_long in (False, True):
      cp = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint(), [], alpha_long, False, False)
      self.assertFalse(cp.alphaLongitudinalAvailable)
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
    self.assertTrue(state.cruiseState.available)
    self.assertFalse(state.cruiseState.enabled)
    self.assertFalse(state.cruiseState.standstill)

  def test_hybrid_subtype_detection_uses_diagnostic_architecture(self):
    # Exact EPS F181 is shared across ICE/HV. GTS differentiates the HV install
    # set by category 466 Brake Booster, while the hybrid controller and 0x127
    # remain independent positive fingerprints.
    for ecu in (Ecu.hybrid, Ecu.electricBrakeBooster):
      with self.subTest(ecu=ecu):
        fw = structs.CarParams.CarFw()
        fw.ecu = ecu
        cp = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint(hybrid=False), [fw], False, False, False)
        self.assertTrue(cp.flags & ToyotaFlags.HYBRID)
        ci = CarInterface(cp)
        self.assertIn("GEAR_PACKET_HYBRID", ci.can_parsers[Bus.pt].vl)
        self.assertNotIn("TSS3_GEAR_PACKET", ci.can_parsers[Bus.pt].vl)

  def test_tss3_gear_carriers(self):
    ice_cp = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint(hybrid=False), [], False, False, False)
    self.assertFalse(ice_cp.flags & ToyotaFlags.HYBRID)
    ice_ci = CarInterface(ice_cp)
    self.assertIn("TSS3_GEAR_PACKET", ice_ci.can_parsers[Bus.pt].vl)
    self.assertNotIn("GEAR_PACKET_HYBRID", ice_ci.can_parsers[Bus.pt].vl)

    expected = {
      "P": structs.CarState.GearShifter.park,
      "R": structs.CarState.GearShifter.reverse,
      "D": structs.CarState.GearShifter.drive,
    }
    for name, want in expected.items():
      state = None
      for i in range(20):
        state = ice_ci.update([(1_000_000_000 + i * 10_000_000,
                                state_packets(hybrid=False, gear=ALBINO_GEAR[name]))])
      self.assertEqual(state.gearShifter, want)

    # N was not exercised in the retained route, but the fourth one-hot value
    # completes P/R/N/D and is independently corroborated by Toyota GTS+ shift ordering.
    neutral = bytearray(ALBINO_GEAR["D"])
    neutral[0] = 0x20
    state = None
    for i in range(20):
      state = ice_ci.update([(2_000_000_000 + i * 10_000_000,
                              state_packets(hybrid=False, gear=bytes(neutral)))])
    self.assertEqual(state.gearShifter, structs.CarState.GearShifter.neutral)

  def test_native_acc_state_standstill_and_set_speed(self):
    ci = CarInterface(self.CP)
    state = update_control_state(ci, set_speed_mph=25)
    self.assertTrue(state.cruiseState.available)
    self.assertTrue(state.cruiseState.enabled)
    self.assertFalse(state.cruiseState.standstill)
    self.assertAlmostEqual(state.cruiseState.speed, 25 * 0.44704, places=6)
    self.assertAlmostEqual(state.cruiseState.speedCluster, state.cruiseState.speed)

    hold = bytearray(SPAN_ACC_ACTIVE)
    hold[7] = 0x67  # request-B ID25 + Brake Only allocation
    state = update_control_state(ci, moving=False, counter_offset=20, acc_frame=bytes(hold), set_speed_mph=25)
    self.assertTrue(state.cruiseState.enabled)
    self.assertTrue(state.cruiseState.standstill)

    hold_override = bytearray(SPAN_ACC_ACTIVE)
    hold_override[7] = 0x66  # request-B ID25 + Engine and Brake 2
    state = update_control_state(ci, moving=False, counter_offset=40, acc_frame=bytes(hold_override), set_speed_mph=25)
    self.assertTrue(state.cruiseState.standstill)

    moving_id25 = bytearray(SPAN_ACC_ACTIVE)
    moving_id25[7] = 0x65  # same ID25, but allocation method 1 is not the delayed hold state
    state = update_control_state(ci, moving=True, counter_offset=60, acc_frame=bytes(moving_id25), set_speed_mph=25)
    self.assertFalse(state.cruiseState.standstill)

  def test_request_plane_and_controller_sideband(self):
    # Span's active 0x08A frame uses the same longitudinal request-plane
    # geometry recovered on Camry: two request IDs/allocation methods and two
    # signed 0.001 m/s^2 acceleration bounds. Keep this decoded even though
    # openpilot does not yet own the source/signing path needed to transmit it.
    request_parser = CANParser("toyota_tss3_pt_generated", [("TSS3_CONTROL_REQUEST", 0)], 1)
    request_parser.update([(1_000_000_000, [CanData(0x08A, SPAN_ACC_ACTIVE, 1)])])
    request = request_parser.vl["TSS3_CONTROL_REQUEST"]
    self.assertEqual(request["LONGITUDINAL_REQUEST_ID_A"], 17)
    self.assertEqual(request["LONGITUDINAL_ALLOCATION_METHOD_A"], 3)
    self.assertEqual(request["LONGITUDINAL_REQUEST_ID_B"], 23)
    self.assertEqual(request["LONGITUDINAL_ALLOCATION_METHOD_B"], 1)
    self.assertAlmostEqual(request["LONGITUDINAL_REQUEST_ACCEL_A"], 0.082, places=6)
    self.assertAlmostEqual(request["LONGITUDINAL_REQUEST_ACCEL_B"], 0.082, places=6)

    # 0x160 stays in the DBC as observed FRC state/evidence, not as a writable
    # longitudinal command contract.
    state_parser = CANParser("toyota_tss3_pt_generated", [("TSS3_FRC_STATE_160", 0)], 2)
    state_parser.update([(1_010_000_000, [CanData(0x160, COROLLA_LONG, 2)])])
    frc_state = state_parser.vl["TSS3_FRC_STATE_160"]
    self.assertEqual(frc_state["COUNTER"], COROLLA_LONG[2])
    self.assertEqual(frc_state["BYTE_4"], COROLLA_LONG[4])
    self.assertEqual(frc_state["BYTE_5"], COROLLA_LONG[5])

    packer = CANPacker("toyota_tss3_pt_generated")
    parser = CANParser("toyota_tss3_pt_generated", [("TSS3_LATERAL_CONTROL", float("nan"))], 1)
    msg = packer.make_can_msg("TSS3_LATERAL_CONTROL", 1, {
      "TARGET_LATERAL_ID": 11, "TARGET_STEERING_ANGLE": 1024 / 17870, "SEQUENCE": 63,
    })
    parser.update([(1_020_000_000, [msg])])
    self.assertEqual(parser.vl["TSS3_LATERAL_CONTROL"]["TARGET_LATERAL_ID"], 11)
    self.assertEqual(parser.vl["TSS3_LATERAL_CONTROL"]["SEQUENCE"], 63)
    self.assertTrue(math.isclose(parser.vl["TSS3_LATERAL_CONTROL"]["TARGET_STEERING_ANGLE"], 1024 / 17870,
                                 rel_tol=0, abs_tol=1e-6))

    ci = CarInterface(self.CP)
    update_control_state(ci)
    output, sends = ci.apply(control(5.0, accel=-1.3, long_active=True), 2_000_000_000)
    self.assertEqual(len(sends), 1)
    address, data, bus = sends[0]
    self.assertEqual((address, bus, len(data)), (0x777, 1, 8))
    self.assertEqual(data[:4], b"\x07\xC7\xC7\x01")
    self.assertEqual(data[6:], b"\x00\x00")
    self.assertFalse(any(address == 0x160 for address, _, _ in sends))
    self.assertEqual(output.accel, 0.0)


class TestToyotaCorollaTSS3Safety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    param = (EPS_SCALE[CAR.TOYOTA_COROLLA_TSS3] | ToyotaSafetyFlags.STOCK_LONGITUDINAL |
             ToyotaSafetyFlags.TSS3_SIGNER | ToyotaSafetyFlags.COROLLA_HF)
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()
    neutral = dict(SPAN_FRAMES)
    neutral[0x116] = bytes(8)
    neutral[0x101] = bytes(8)
    for address in (0x025, 0x030, 0x0AA, 0x116, 0x101):
      self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, 1, neutral[address])))
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 1, SPAN_ACC_ACTIVE)))
    self.assertTrue(self.safety.get_controls_allowed())

  @staticmethod
  def c7(angle_raw=0, sequence=1, bus=1):
    data = b"\x07\xC7\xC7" + bytes((sequence,)) + angle_raw.to_bytes(2, "big", signed=True) + b"\x00\x00"
    return libsafety_py.make_CANPacket(0x777, bus, data)

  def test_only_bounded_c7_on_stock_toyota_b(self):
    self.assertTrue(self.safety.safety_tx_hook(self.c7()))
    self.assertFalse(self.safety.safety_tx_hook(self.c7(bus=0)))
    # 0x777 remains a diagnostic address in stock firmware; Panda grants only
    # the exact private C7 envelope, never arbitrary functional diagnostics.
    self.assertFalse(self.safety.safety_tx_hook(
      libsafety_py.make_CANPacket(0x777, 1, bytes.fromhex("0210030000000000"))))
    self.assertFalse(self.safety.safety_tx_hook(self.c7(1746)))
    for index in (0, 1, 2, 6, 7):
      data = bytearray(self.c7()[0].data)
      data[index] ^= 1
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x777, 1, bytes(data))))

  def test_0x160_is_not_host_replaceable(self):
    for bus in range(3):
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x160, bus, COROLLA_LONG)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x160), 0)
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x08A, 1, SPAN_ACC_ACTIVE)))

  def test_native_cruise_gate_revokes_control(self):
    disabled = bytearray(SPAN_ACC_ACTIVE)
    disabled[22] &= ~0x10
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 1, bytes(disabled))))
    self.assertFalse(self.safety.get_controls_allowed())

    legacy_active = bytearray(SPAN_FRAMES[0x176])
    legacy_active[0] |= 0x20
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x176, 1, bytes(legacy_active))))
    self.assertFalse(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
