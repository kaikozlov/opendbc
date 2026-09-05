import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, CanData, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.toyota.fingerprints import FINGERPRINTS, FW_VERSIONS, TSS3_CAN_CENSUS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC, EPS_SCALE, ToyotaFlags, ToyotaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py

Ecu = structs.CarParams.Ecu
ButtonType = structs.CarState.ButtonEvent.Type

CAMRY_COMMON = {
  0x00F: bytes.fromhex("01b20145cde4b47d"),
  0x025: bytes.fromhex("000100005000007e0000000000000000000000000000000000000000bb6fee54"),
  0x08A: bytes.fromhex("0000000880002d47fe462afe467fff007fffff35c000100064003c005db7797f"),
  0x030: bytes.fromhex("00000000170000500000100026820000000000010000ffff00000000b280595f"),
  0x0AA: bytes.fromhex("1a6f1a6f1a6f1a6f"),
  0x0FE: bytes.fromhex("567d393f0000c36200000000000000002640000000ff000000000000d54aaf10"),
  0x101: bytes.fromhex("800000010000008b"),
  0x116: bytes.fromhex("000000007b4b235a"),
  0x251: bytes.fromhex("c01015908030a080"),
  0x610: bytes.fromhex("00001d4ed0fffc00"),
  0x51E: bytes.fromhex("80006e0000000000"),
  0x614: bytes.fromhex("00004a3000003303"),
  0x620: bytes.fromhex("000000008000001a"),
}
CAMRY_GEAR = {
  structs.CarState.GearShifter.park: bytes.fromhex("00100000000ebe0c"),
  structs.CarState.GearShifter.reverse: bytes.fromhex("00100000001e8deb"),
  structs.CarState.GearShifter.neutral: bytes.fromhex("00100000002e8dfb"),
  structs.CarState.GearShifter.drive: bytes.fromhex("00100000003e8d0b"),
  structs.CarState.GearShifter.brake: bytes.fromhex("00100000004e8d1b"),
}


def fingerprint() -> dict[int, dict[int, int]]:
  fp = {i: {} for i in range(8)}
  fp[0] = {0x025: 32, 0x0AA: 8, 0x3F6: 8}
  return fp


def update_with_frame_set(ci: CarInterface, frames: dict[int, bytes], repeats: int = 20):
  packet = [CanData(address, dat, 2 if address in (0x08A, 0x251) else 0) for address, dat in frames.items()]
  ret = None
  for i in range(repeats):
    ret = ci.update([(1_000_000_000 + i * 10_000_000, packet)])
  return ret


def control(angle_deg: float, lat_active: bool = True, cancel: bool = False):
  cc = structs.CarControl()
  cc.enabled = True
  cc.latActive = lat_active
  cc.cruiseControl.cancel = cancel
  cc.actuators.steeringAngleDeg = angle_deg
  return cc.as_reader()


class TestToyotaCamryTSS3Platform(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], False, False, False)

  def test_platform_uses_normal_toyota_control_path(self):
    self.assertTrue(self.CP.flags & ToyotaFlags.TSS3)
    self.assertTrue(self.CP.flags & ToyotaFlags.SECOC)
    self.assertFalse(self.CP.flags & ToyotaFlags.TSS2)
    self.assertFalse(self.CP.flags & ToyotaFlags.NO_DSU)
    self.assertEqual(DBC[CAR.TOYOTA_CAMRY_TSS3][Bus.pt], "toyota_tss3_pt_generated")
    self.assertFalse(self.CP.dashcamOnly)
    self.assertEqual(self.CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.toyota)
    self.assertTrue(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.TSS3)
    self.assertTrue(self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL)
    self.assertFalse(self.CP.secOcRequired)
    self.assertEqual(self.CP.steerControlType, structs.CarParams.SteerControlType.angle)
    self.assertFalse(self.CP.openpilotLongitudinalControl)
    self.assertEqual(self.CP.minEnableSpeed, -1.0)
    self.assertTrue(self.CP.steerAtStandstill)
    self.assertEqual(self.CP.minSteerSpeed, 0.)
    self.assertTrue(self.CP.enableBsm)

  def test_identity_uses_standard_firmware_and_can_tables(self):
    fw = FW_VERSIONS[CAR.TOYOTA_CAMRY_TSS3]
    self.assertEqual(fw[(Ecu.eps, 0x7A1, None)], [
      bytes.fromhex("023839363546333330373030300000000038413331313333303331303000000000")])
    self.assertEqual(fw[(Ecu.fwdCamera, 0x792, None)], [bytes.fromhex("0138363436463333313530303000000000")])
    self.assertEqual(fw[(Ecu.abs, 0x7B0, None)], [bytes.fromhex("01463135323633334b3030303000000000")])
    self.assertEqual(FINGERPRINTS[CAR.TOYOTA_CAMRY_TSS3][0], TSS3_CAN_CENSUS[CAR.TOYOTA_CAMRY_TSS3])

  def test_carstate_uses_fixed_relay_topology_and_stock_acc_state(self):
    ci = CarInterface(self.CP)
    cs = update_with_frame_set(ci, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    self.assertEqual(cs.gearShifter, structs.CarState.GearShifter.drive)
    self.assertTrue(cs.cruiseState.available)
    self.assertTrue(cs.cruiseState.enabled)
    self.assertFalse(cs.carNotReady)
    self.assertAlmostEqual(cs.cruiseState.speed, CAMRY_COMMON[0x08A][10] * CV.KPH_TO_MS, places=5)
    self.assertAlmostEqual(cs.cruiseState.speedCluster, CAMRY_COMMON[0x251][2] * CV.MPH_TO_MS, places=5)

    packer = CANPacker(DBC[CAR.TOYOTA_CAMRY_TSS3][Bus.pt])
    _, metric_units, _ = packer.make_can_msg("BODY_CONTROL_STATE_2", 0, {"UNITS": 1})
    _, metric_display, _ = packer.make_can_msg("TSS3_CRUISE_DISPLAY", 0, {"UI_SET_SPEED": 34})
    cs = update_with_frame_set(ci, CAMRY_COMMON | {0x610: metric_units, 0x251: metric_display,
                                                   0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    self.assertAlmostEqual(cs.cruiseState.speedCluster, 34 * CV.KPH_TO_MS, places=5)

    cruise_off = bytearray(CAMRY_COMMON[0x08A])
    cruise_off[3] &= ~0x08
    cruise_off[10] = 0
    cs = update_with_frame_set(ci, CAMRY_COMMON | {0x08A: bytes(cruise_off), 0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    self.assertFalse(cs.cruiseState.available)
    self.assertFalse(cs.cruiseState.enabled)

  def test_carstate_exposes_stock_cruise_button_events(self):
    ci = CarInterface(self.CP)
    base = CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]}
    update_with_frame_set(ci, base)
    packer = CANPacker(DBC[CAR.TOYOTA_CAMRY_TSS3][Bus.pt])

    cases = [
      ({"MAIN_BUTTON": 1, "RES_BUTTON_MIRROR_N": 1, "SET_BUTTON_MIRROR_N": 1, "CANCEL_BUTTON_MIRROR_N": 1}, ButtonType.mainCruise),
      ({"RES_BUTTON": 1, "RES_BUTTON_MIRROR_N": 0, "SET_BUTTON_MIRROR_N": 1, "CANCEL_BUTTON_MIRROR_N": 1}, ButtonType.accelCruise),
      ({"SET_BUTTON": 1, "RES_BUTTON_MIRROR_N": 1, "SET_BUTTON_MIRROR_N": 0, "CANCEL_BUTTON_MIRROR_N": 1}, ButtonType.decelCruise),
      ({"CANCEL_BUTTON": 1, "RES_BUTTON_MIRROR_N": 1, "SET_BUTTON_MIRROR_N": 1, "CANCEL_BUTTON_MIRROR_N": 0}, ButtonType.cancel),
    ]
    for values, expected in cases:
      _, msg, _ = packer.make_can_msg("TSS3_CRUISE_SWITCH", 0, values)
      cs = update_with_frame_set(ci, base | {0x0FE: msg}, repeats=1)
      self.assertEqual([(e.type, e.pressed) for e in cs.buttonEvents], [(expected, True)])
      cs = update_with_frame_set(ci, base, repeats=1)
      self.assertEqual([(e.type, e.pressed) for e in cs.buttonEvents], [(expected, False)])

  def test_carstate_exposes_standard_toyota_body_chassis_and_bsm_state(self):
    ci = CarInterface(self.CP)
    packer = CANPacker(DBC[CAR.TOYOTA_CAMRY_TSS3][Bus.pt])
    _, body, _ = packer.make_can_msg("BODY_CONTROL_STATE", 0, {
      "DOOR_OPEN_FL": 1,
      "SEATBELT_DRIVER_UNLATCHED": 1,
      "PARKING_BRAKE": 1,
    })
    _, esp, _ = packer.make_can_msg("ESP_CONTROL", 0, {"BRAKE_HOLD_ACTIVE": 1, "TC_DISABLED": 1})
    _, stalk, _ = packer.make_can_msg("LIGHT_STALK", 0, {"AUTO_HIGH_BEAM": 1})
    _, bsm, _ = packer.make_can_msg("BSM", 0, {"L_ADJACENT": 1, "R_APPROACHING": 1})
    cs = update_with_frame_set(ci, CAMRY_COMMON | {
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
      0x620: body,
      0x3B7: esp,
      0x622: stalk,
      0x3F6: bsm,
    })
    self.assertTrue(cs.doorOpen)
    self.assertTrue(cs.seatbeltUnlatched)
    self.assertTrue(cs.parkingBrake)
    self.assertTrue(cs.brakeHoldActive)
    self.assertTrue(cs.espDisabled)
    self.assertTrue(cs.genericToggle)
    self.assertTrue(cs.leftBlindspot)
    self.assertTrue(cs.rightBlindspot)

  def test_carstate_driver_intervention_and_sensor_validity(self):
    ci = CarInterface(self.CP)
    packer = CANPacker(DBC[CAR.TOYOTA_CAMRY_TSS3][Bus.pt])
    base = CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]}

    def eps_msg(coarse: float, invalid: int = 0) -> bytes:
      _, msg, _ = packer.make_can_msg("TSS3_EPS_TELEMETRY", 0, {
        "STEERING_WHEEL_TORQUE_COARSE": coarse,
        "STEERING_WHEEL_TORQUE_FINE": 0.0,
        "DRIVER_TORQUE_INVALID": invalid,
        "STEERING_FAULT_INHIBIT_STATUS": 1,
      })
      return msg

    for torque, pressed in ((2.0, True), (-2.0, True), (0.5, False), (-0.5, False)):
      with self.subTest(torque=torque):
        cs = update_with_frame_set(ci, base | {0x030: eps_msg(torque)})
        self.assertAlmostEqual(cs.steeringTorque, torque)
        self.assertEqual(cs.steeringPressed, pressed)
        self.assertFalse(cs.vehicleSensorsInvalid)

    cs = update_with_frame_set(ci, base | {0x030: eps_msg(2.0, invalid=1)})
    self.assertAlmostEqual(cs.steeringTorque, 0.0)
    self.assertFalse(cs.steeringPressed)

    self.assertTrue(cs.vehicleSensorsInvalid)

    cs = update_with_frame_set(ci, base | {0x030: eps_msg(2.0)})
    self.assertTrue(cs.steeringPressed)
    self.assertFalse(cs.vehicleSensorsInvalid)

    _, wheel_fault, _ = packer.make_can_msg("WHEEL_SPEEDS", 0, {"WHEEL_SPEED_FL_FAULT": 1})
    cs = update_with_frame_set(ci, base | {0x030: eps_msg(2.0), 0x0AA: wheel_fault})
    self.assertTrue(cs.vehicleSensorsInvalid)

  def test_controller_sends_clean_b6_like_a_normal_angle_port(self):
    ci = CarInterface(self.CP)
    update_with_frame_set(ci, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    output, sends = ci.apply(control(5.0), 2_000_000_000)
    self.assertEqual(len(sends), 1)
    addr, dat, bus = sends[0]
    self.assertEqual((addr, bus, len(dat)), (0x0B6, 0, 32))
    self.assertEqual(dat[3] & 0x3F, 11)
    self.assertEqual(dat[6] & 0x04, 0)
    self.assertEqual(dat[8:10], b"\x64\x64")
    self.assertEqual(int.from_bytes(dat[28:32], "big") & 0x0FFFFFFF, 0)
    self.assertAlmostEqual(output.steeringAngleDeg,
                           int.from_bytes(dat[4:6], "big", signed=True) * (1024 / 17870), delta=0.03)
    self.assertFalse(any(addr == 0x08A for addr, _, _ in sends))

  def test_controller_inactive_b6_tracks_measured_angle(self):
    ci = CarInterface(self.CP)
    cs = update_with_frame_set(ci, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    _, sends = ci.apply(control(20.0, lat_active=False), 2_000_000_000)
    dat = next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0)
    self.assertEqual(dat[3] & 0x3F, 0)
    self.assertEqual(dat[6] & 0x04, 0x04)
    self.assertEqual(dat[8:10], b"\x00\x00")
    commanded_deg = int.from_bytes(dat[4:6], "big", signed=True) * (1024 / 17870)
    self.assertAlmostEqual(commanded_deg, cs.steeringAngleDeg, delta=0.12)

  def test_b6_sequence_and_zero_marker_freshness_progress(self):
    ci = CarInterface(self.CP)
    update_with_frame_set(ci, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    _, sends = ci.apply(control(1.0), 2_000_000_000)
    first = next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0)
    ci.apply(control(1.0), 2_010_000_000)
    _, sends = ci.apply(control(1.0), 2_020_000_000)
    second = next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0)

    self.assertEqual((second[7] - first[7]) & 0x3F, 1)
    self.assertEqual(((second[28] >> 6) - (first[28] >> 6)) & 0x3, 1)
    self.assertEqual(int.from_bytes(first[28:32], "big") & 0x0FFFFFFF, 0)
    self.assertEqual(int.from_bytes(second[28:32], "big") & 0x0FFFFFFF, 0)

  def test_b6_freshness_counter_reanchors_on_sync_reset(self):
    ci = CarInterface(self.CP)
    base = CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]}
    update_with_frame_set(ci, base)
    _, sends = ci.apply(control(1.0), 2_000_000_000)
    first = next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0)
    ci.apply(control(1.0), 2_010_000_000)
    _, sends = ci.apply(control(1.0), 2_020_000_000)
    second = next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0)

    sync = bytearray(CAMRY_COMMON[0x00F])
    reset = (sync[2] << 12) | (sync[3] << 4) | (sync[4] >> 4)
    reset = (reset + 1) & 0xFFFFF
    sync[2] = (reset >> 12) & 0xFF
    sync[3] = (reset >> 4) & 0xFF
    sync[4] = (sync[4] & 0x0F) | ((reset & 0x0F) << 4)
    update_with_frame_set(ci, base | {0x00F: bytes(sync)}, repeats=1)
    ci.apply(control(1.0), 2_030_000_000)
    _, sends = ci.apply(control(1.0), 2_040_000_000)
    reanchored = next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0)

    self.assertEqual((first[28] >> 6, second[28] >> 6), (0, 1))
    self.assertEqual(reanchored[28] >> 6, 0)
    self.assertEqual((reanchored[28] >> 4) & 0x3, reset & 0x3)

  def test_controller_brake_cancel_clones_stock_101(self):
    ci = CarInterface(self.CP)
    stock_brake = bytes.fromhex("8000000600000090")
    update_with_frame_set(ci, CAMRY_COMMON | {
      0x101: stock_brake,
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
    })
    _, sends = ci.apply(control(1.0, cancel=True), 2_000_000_000)
    brake_cancel = [m for m in sends if m[0] == 0x101]
    self.assertEqual(brake_cancel, [(0x101, bytes.fromhex("8800000600000098"), 2)])


class TestToyotaCamryTSS3PandaSafety(unittest.TestCase):
  def setUp(self):
    self.s = libsafety_py.libsafety
    param = EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.TSS3
    self.assertEqual(self.s.set_safety_hooks(structs.CarParams.SafetyModel.toyota, param), 0)
    self.s.init_tests()

    for addr in (0x025, 0x030, 0x0AA, 0x116, 0x101, 0x00F):
      self.assertTrue(self.s.safety_rx_hook(libsafety_py.make_CANPacket(addr, 0, CAMRY_COMMON[addr])))
    self.assertTrue(self.s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, CAMRY_COMMON[0x08A])))
    self.assertTrue(self.s.get_controls_allowed())

    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], False, False, False)
    self.ci = CarInterface(cp)
    update_with_frame_set(self.ci, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})

  def test_08a_is_not_a_camry_tx_object(self):
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x08A, 0, CAMRY_COMMON[0x08A])))

  def next_b6(self, angle=1.0, active=True):
    _, sends = self.ci.apply(control(angle, active), 2_000_000_000)
    if not any(addr == 0x0B6 for addr, _, _ in sends):
      _, sends = self.ci.apply(control(angle, active), 2_010_000_000)
    return bytearray(next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0))

  def reset_safety(self, controls=True):
    self.s.init_tests()
    for addr in (0x025, 0x030, 0x0AA, 0x116, 0x101, 0x00F):
      self.s.safety_rx_hook(libsafety_py.make_CANPacket(addr, 0, CAMRY_COMMON[addr]))
    a8 = CAMRY_COMMON[0x08A]
    if not controls:
      off = bytearray(a8)
      off[3] &= ~0x08
      a8 = bytes(off)
    self.s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, a8))

  def test_normal_angle_safety_accepts_controller_b6(self):
    dat = self.next_b6()
    self.assertTrue(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x0B6, 0, bytes(dat))))

  def test_b6_safety_rejects_wrong_mode_overangle_and_controls_off(self):
    dat = self.next_b6()
    bad_mode = bytearray(dat)
    bad_mode[3] = (bad_mode[3] & 0xC0) | 4
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x0B6, 0, bytes(bad_mode))))

    self.reset_safety()
    overangle = bytearray(self.next_b6())
    overangle[4:6] = (1746).to_bytes(2, "big", signed=True)
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x0B6, 0, bytes(overangle))))

    self.reset_safety(controls=False)
    self.assertFalse(self.s.get_controls_allowed())
    dat = self.next_b6()
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x0B6, 0, bytes(dat))))

  def test_b6_safety_does_not_promote_companion_fields_to_policy(self):
    dat = self.next_b6()
    dat[8] = 0
    dat[9] = 0
    dat[6] ^= 0x04
    self.assertTrue(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x0B6, 0, bytes(dat))))

  def test_stock_acc_owns_controls_allowed(self):
    off = bytearray(CAMRY_COMMON[0x08A])
    off[3] &= ~0x08
    self.assertTrue(self.s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, bytes(off))))
    self.assertFalse(self.s.get_controls_allowed())
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x08A, 0, CAMRY_COMMON[0x08A])))

  def test_brake_cancel_safety_allows_only_stock_shaped_checked_frame(self):
    good = bytes.fromhex("8800000600000098")
    self.assertTrue(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, good)))

    brake_off = bytearray(good)
    brake_off[0] &= ~0x08
    brake_off[7] = (8 + 1 + 1 + sum(brake_off[:7])) & 0xFF
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, bytes(brake_off))))

    bad_shape = bytearray(good)
    bad_shape[4] = 1
    bad_shape[7] = (8 + 1 + 1 + sum(bad_shape[:7])) & 0xFF
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, bytes(bad_shape))))

    bad_checksum = bytearray(good)
    bad_checksum[7] ^= 1
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, bytes(bad_checksum))))

    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 0, good)))

  def test_relay_forwards_stock_08a_and_blocks_stock_b6_replacement(self):
    self.assertEqual(self.s.safety_fwd_hook(2, 0x08A), 0)
    self.assertEqual(self.s.safety_fwd_hook(2, 0x0B6), -1)


if __name__ == "__main__":
  unittest.main()
