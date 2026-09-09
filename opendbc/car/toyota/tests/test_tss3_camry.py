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
  0x3B7: bytes.fromhex("0000000020000008"),
  0x3F6: bytes.fromhex("81ea6e0480ba4808"),
  0x412: bytes.fromhex("1400004401ee9307"),
  0x610: bytes.fromhex("00001d4ed0fffc00"),
  0x51E: bytes.fromhex("80006e0000000000"),
  0x614: bytes.fromhex("00004a3000003303"),
  0x620: bytes.fromhex("000000008000001a"),
  0x622: bytes.fromhex("0000000000730000"),
}

# Native source/cadence from the relay-correct September Camry captures. The
# decoder helper below intentionally does not model cadence; integration tests
# use replay_native() so topology/liveness cannot be hidden by synthetic 100 Hz
# traffic.
CAMRY_NATIVE_BUS = {
  0x08A: 2, 0x251: 2, 0x3F6: 2, 0x412: 2,
}
CAMRY_NATIVE_HZ = {
  0x00F: 10, 0x025: 100, 0x030: 100, 0x08A: 40, 0x0AA: 100,
  0x0FE: 30, 0x101: 50, 0x116: 40, 0x127: 50, 0x251: 1,
  0x3B7: 3, 0x3F6: 1, 0x412: 1, 0x51E: 1, 0x610: 3,
  0x614: 1, 0x620: 3, 0x622: 1,
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
  fp[0] = {0x025: 32, 0x0AA: 8}
  fp[2] = {0x3F6: 8}
  return fp


def update_with_frame_set(ci: CarInterface, frames: dict[int, bytes], repeats: int = 20):
  """Field-decoder helper: monotonic timestamps and native buses, not cadence."""
  packet = [CanData(address, dat, CAMRY_NATIVE_BUS.get(address, 0)) for address, dat in frames.items()]
  now = getattr(ci, "_tss3_test_now_nanos", 1_000_000_000)
  ret = None
  for _ in range(repeats):
    now += 10_000_000
    ret = ci.update([(now, packet)])
  ci._tss3_test_now_nanos = now
  return ret


def replay_at_native_cadence(ci: CarInterface, frames: dict[int, bytes], duration_s: float, omit: set[int] | None = None):
  """Schedule source-real payloads synthetically at measured native buses/cadences."""
  omit = omit or set()
  now = getattr(ci, "_tss3_replay_now_nanos", 1_000_000_000)
  next_due = getattr(ci, "_tss3_replay_next_due", {})
  ret = None
  end = now + int(duration_s * 1e9)
  while now <= end:
    packet = []
    for address, dat in frames.items():
      hz = CAMRY_NATIVE_HZ[address]
      due = next_due.get(address, now)
      if now >= due:
        if address not in omit:
          packet.append(CanData(address, dat, CAMRY_NATIVE_BUS.get(address, 0)))
        period = int(1e9 / hz)
        while due <= now:
          due += period
        next_due[address] = due
    ret = ci.update([(now, packet)])
    now += 10_000_000
  ci._tss3_replay_now_nanos = now
  ci._tss3_replay_next_due = next_due
  return ret


def control(angle_deg: float, lat_active: bool = True, cancel: bool = False,
            left_lane: bool = False, right_lane: bool = False, steer_alert: bool = False):
  cc = structs.CarControl()
  cc.enabled = True
  cc.latActive = lat_active
  cc.cruiseControl.cancel = cancel
  cc.actuators.steeringAngleDeg = angle_deg
  cc.hudControl.leftLaneVisible = left_lane
  cc.hudControl.rightLaneVisible = right_lane
  if steer_alert:
    cc.hudControl.visualAlert = structs.CarControl.HUDControl.VisualAlert.steerRequired
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
    # The fixture is physically stopped, but stock ACC is still in its ordinary
    # pre-hold state (0x08A B7=0x47), so resume-required standstill is false.
    self.assertFalse(cs.cruiseState.standstill)
    self.assertFalse(cs.carNotReady)
    self.assertAlmostEqual(cs.cruiseState.speed, CAMRY_COMMON[0x08A][10] * CV.KPH_TO_MS, places=5)
    self.assertAlmostEqual(cs.cruiseState.speedCluster, CAMRY_COMMON[0x251][2] * CV.MPH_TO_MS, places=5)

    packer = CANPacker(DBC[CAR.TOYOTA_CAMRY_TSS3][Bus.pt])
    _, metric_units, _ = packer.make_can_msg("BODY_CONTROL_STATE_2", 0, {"UNITS": 1})
    _, metric_display, _ = packer.make_can_msg("TSS3_CRUISE_DISPLAY", 0, {"UI_SET_SPEED": 34})
    cs = update_with_frame_set(ci, CAMRY_COMMON | {0x610: metric_units, 0x251: metric_display,
                                                   0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    self.assertAlmostEqual(cs.cruiseState.speedCluster, 34 * CV.KPH_TO_MS, places=5)

    # Source-real route-3b stock-ACC hold states. 0x67 is the ordinary held
    # standstill state; 0x66 is its accelerator-override companion just before
    # the state clears. Both occur at exact zero speed after a delayed stop.
    stock_hold = bytes.fromhex("00000008a0002d67fe703bfe707fff007ffffff7400b10006400350029b3a489")
    cs = update_with_frame_set(ci, CAMRY_COMMON | {0x08A: stock_hold,
                                                   0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    self.assertTrue(cs.cruiseState.standstill)
    stock_hold_gas = bytes.fromhex("00000008a0002c66fe703bfe707fff007ffffff7400b100064002b008676d649")
    cs = update_with_frame_set(ci, CAMRY_COMMON | {0x08A: stock_hold_gas,
                                                   0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    self.assertTrue(cs.cruiseState.standstill)

    # B7 bit5 alone is not the contract: this source-real moving transition has
    # B7=0x65 and must not be classified as stock-ACC standstill.
    moving_transition = bytes.fromhex("0000000880044765fda800fda87fff007fff0030000b100064003b006acb6f13")
    cs = update_with_frame_set(ci, CAMRY_COMMON | {0x08A: moving_transition,
                                                   0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    self.assertFalse(cs.cruiseState.standstill)

    cruise_off = bytearray(CAMRY_COMMON[0x08A])
    cruise_off[3] &= ~0x08
    cruise_off[10] = 0
    # Original same-car CANCEL state: operation clears while 0x251 B1[4]
    # preserves cruise main/availability.
    cancelled_display = bytes.fromhex("a01015488028a080")
    cs = update_with_frame_set(ci, CAMRY_COMMON | {
      0x08A: bytes(cruise_off),
      0x251: cancelled_display,
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
    })
    self.assertTrue(cs.cruiseState.available)
    self.assertFalse(cs.cruiseState.enabled)

    # Before the first effective MAIN press, the same carrier has B1[4]=0.
    pre_main_display = bytes.fromhex("a00000488028a080")
    cs = update_with_frame_set(ci, CAMRY_COMMON | {
      0x08A: bytes(cruise_off),
      0x251: pre_main_display,
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
    })
    self.assertFalse(cs.cruiseState.available)
    self.assertFalse(cs.cruiseState.enabled)

  def test_carstate_uses_source_real_cluster_speed(self):
    ci = CarInterface(self.CP)
    # Coherent native route-2c pair: 0x610 UI_SPEED=29 km/h and the preceding
    # 0x0AA wheel-speed frame 9.93 ms earlier while the car is moving.
    frames = CAMRY_COMMON | {
      0x610: bytes.fromhex("00001d4ed0008c00"),
      0x0AA: bytes.fromhex("25a0259525902581"),
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
    }
    cs = update_with_frame_set(ci, frames)
    self.assertGreater(cs.vEgo, 1.0)
    # CarInterfaceBase applies the normal 0.5 km/h cluster-speed hysteresis.
    self.assertAlmostEqual(cs.vEgoCluster, (29 - 0.5) * CV.KPH_TO_MS, places=5)

  def test_periodic_inputs_invalidate_and_recover_at_native_cadence(self):
    base = CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]}
    # Every parser-checked source must independently invalidate and recover.
    # The rates are conservative floors of the measured native cadence, not
    # synthetic target frequencies for the vehicle.
    for address in CAMRY_NATIVE_HZ:
      with self.subTest(address=hex(address)):
        ci = CarInterface(self.CP)
        cs = replay_at_native_cadence(ci, base, 1.1)
        self.assertTrue(cs.canValid)

        timeout_s = 10 / CAMRY_NATIVE_HZ[address]
        cs = replay_at_native_cadence(ci, base, timeout_s + 0.2, omit={address})
        self.assertFalse(cs.canValid)

        cs = replay_at_native_cadence(ci, base, max(0.2, 1 / CAMRY_NATIVE_HZ[address] + 0.05))
        self.assertTrue(cs.canValid)

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

    def eps_msg(coarse: float, fine: float = 0.0, invalid: int = 0, steering_inhibit: int = 0) -> bytes:
      _, msg, _ = packer.make_can_msg("TSS3_EPS_TELEMETRY", 0, {
        "STEERING_WHEEL_TORQUE_COARSE": coarse,
        "STEERING_WHEEL_TORQUE_FINE": fine,
        "DRIVER_TORQUE_INVALID": invalid,
        "STEERING_FAULT_INHIBIT_STATUS": steering_inhibit,
      })
      return msg

    for torque, pressed in ((2.0, True), (-2.0, True), (0.7, True), (-0.7, True),
                            (0.6, True), (-0.6, True), (0.5, False), (-0.5, False)):
      with self.subTest(torque=torque):
        cs = update_with_frame_set(ci, base | {0x030: eps_msg(torque)})
        self.assertAlmostEqual(cs.steeringTorque, torque)
        self.assertEqual(cs.steeringPressed, pressed)
        self.assertFalse(cs.vehicleSensorsInvalid)
        self.assertFalse(cs.steerFaultTemporary)
        self.assertFalse(cs.steerFaultPermanent)

    cs = update_with_frame_set(ci, base | {0x030: eps_msg(0.6, fine=0.01)})
    self.assertAlmostEqual(cs.steeringTorque, 0.61, places=2)
    self.assertTrue(cs.steeringPressed)

    # The raw selected steering fault/inhibit aggregate is intentionally not
    # promoted to temporary/permanent policy without an asserted/recovery join.
    cs = update_with_frame_set(ci, base | {0x030: eps_msg(2.0, steering_inhibit=1)})
    self.assertFalse(cs.steerFaultTemporary)
    self.assertFalse(cs.steerFaultPermanent)

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

  def test_carstate_replays_real_september_2026_0904_eps_frames(self):
    """Real 0x030 wire bytes from the 2026-09-04 highway corpus (VAR-125).

    Frames are native bus-0 captures from routes 0000003d--0e812cecba
    segments 1/2/0; expected N.m values are the exact-F33 packer geometry
    decode (B8 coarse 0.1 + B17[3:0] fine 0.01), independently derived in the
    analysis repo reducer, not from this parser. The revision that produced
    those routes hardcoded steeringPressed=False on every sample, which made
    DesireHelper lane-change entry impossible; this pins the fixed behavior
    against the original wire bytes.
    """
    ci = CarInterface(self.CP)
    base = CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]}
    real_frames = (
      # positive ~driver torque: coarse 42, fine +3 -> 4.23 N.m, valid
      ("2a000013610142192a13601fb6400206020300001fb61b6c00000000edee2c5a", 4.23, True, False),
      # negative driver torque: coarse -29, fine +4 -> -2.86 N.m, valid
      ("e4000001d4e4d2a7e301d002ff40d4f90204000002ffeec200000000a8645038", -2.86, True, False),
      # DRIVER_TORQUE_INVALID (B6[0]) asserted with EPS_STATUS_B6_BIT3
      ("00000000000009410000080000120000000000010000fff50000000054b9bac6", 0.0, False, True),
    )
    for hexdat, torque, pressed, invalid in real_frames:
      with self.subTest(torque=torque):
        cs = update_with_frame_set(ci, base | {0x030: bytes.fromhex(hexdat)})
        self.assertAlmostEqual(cs.steeringTorque, torque, places=2)
        self.assertEqual(cs.steeringPressed, pressed)
        self.assertEqual(cs.vehicleSensorsInvalid, invalid)

  def test_controller_sends_clean_b6_like_a_normal_angle_port(self):
    ci = CarInterface(self.CP)
    update_with_frame_set(ci, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    output, sends = ci.apply(control(5.0), 2_000_000_000)
    b6 = [m for m in sends if m[0] == 0x0B6]
    self.assertEqual(len(b6), 1)
    addr, dat, bus = b6[0]
    self.assertEqual((addr, bus, len(dat)), (0x0B6, 0, 32))
    self.assertEqual(dat[3] & 0x3F, 11)
    self.assertEqual(dat[6] & 0x04, 0)
    self.assertEqual(dat[8:10], b"\x64\x64")
    # B28..B31 are the exact marker consumed by the EPS-resident signer. The
    # host owns application semantics only; EPS owns SecOC freshness and MAC.
    self.assertEqual(dat[28:32], bytes(4))
    self.assertAlmostEqual(output.steeringAngleDeg,
                           int.from_bytes(dat[4:6], "big", signed=True) * (1024 / 17870), delta=0.03)
    self.assertFalse(any(addr == 0x08A for addr, _, _ in sends))

  def test_controller_replaces_stock_hud_without_hands_off_nag(self):
    ci = CarInterface(self.CP)
    stock_hud = bytes.fromhex("140c404401ee9307")
    update_with_frame_set(ci, CAMRY_COMMON | {
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
      0x412: stock_hud,
    })

    _, sends = ci.apply(control(1.0, left_lane=True, right_lane=True), 2_000_000_000)
    hud = [m for m in sends if m[0] == 0x412]
    self.assertEqual(hud, [(0x412, bytes.fromhex("1400004401ee9307"), 0)])

  def test_controller_renders_tss3_lane_visibility_on_stock_hud(self):
    ci = CarInterface(self.CP)
    stock_hud = bytes.fromhex("1200001202ee9307")
    update_with_frame_set(ci, CAMRY_COMMON | {
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
      0x412: stock_hud,
    })

    # Existing road data does not prove whether B3 high/low is left/right.
    # For an asymmetric openpilot request, preserve the stock per-side nibble
    # orientation while converting recognized state 1 -> active state 4.
    _, sends = ci.apply(control(1.0, left_lane=False, right_lane=True), 2_000_000_000)
    hud = next(dat for addr, dat, bus in sends if addr == 0x412 and bus == 0)
    self.assertEqual(hud, bytes.fromhex("1400004201ee9307"))

    # Inactive recognized lines return to state 1 on the same stock-oriented
    # nibble. Display changes are published no faster than the native ~10 Hz
    # event path.
    for _ in range(9):
      ci.apply(control(1.0, lat_active=False, left_lane=False, right_lane=True), 2_010_000_000)
    _, sends = ci.apply(control(1.0, lat_active=False, left_lane=False, right_lane=True), 2_020_000_000)
    hud = next(dat for addr, dat, bus in sends if addr == 0x412 and bus == 0)
    self.assertEqual(hud, stock_hud)

  def test_controller_preserves_noncanonical_hud_mode(self):
    ci = CarInterface(self.CP)
    # Route-3b retained noncanonical road frame: B0 low mode 0 is outside the
    # two recovered road-state shapes and must not be rewritten to the normal
    # inactive lane display.
    stock_hud = bytes.fromhex("1000042102ee9307")
    update_with_frame_set(ci, CAMRY_COMMON | {
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
      0x412: stock_hud,
    })

    _, sends = ci.apply(control(1.0, lat_active=False, left_lane=True, right_lane=False, steer_alert=True), 2_000_000_000)
    hud = next(dat for addr, dat, bus in sends if addr == 0x412 and bus == 0)
    self.assertEqual(hud, stock_hud)

  def test_controller_hud_matches_native_heartbeat_and_openpilot_steer_alert(self):
    ci = CarInterface(self.CP)
    update_with_frame_set(ci, CAMRY_COMMON | {
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
      0x412: bytes.fromhex("140c404401ee9307"),
    })

    # First replacement is immediate. Stable HUD state then follows the native
    # ~1 Hz heartbeat rather than the old 5 Hz ordinary-Toyota cadence.
    _, sends = ci.apply(control(1.0, left_lane=True, right_lane=True), 2_000_000_000)
    self.assertEqual(len([m for m in sends if m[0] == 0x412]), 1)
    hud_count = 0
    for i in range(1, 100):
      _, sends = ci.apply(control(1.0, left_lane=True, right_lane=True), 2_000_000_000 + i * 10_000_000)
      hud_count += len([m for m in sends if m[0] == 0x412])
    self.assertEqual(hud_count, 0)
    _, sends = ci.apply(control(1.0, left_lane=True, right_lane=True), 3_000_000_000)
    self.assertEqual(len([m for m in sends if m[0] == 0x412]), 1)

    # The recovered B1[3:2] hands-off visual is owned by openpilot while the
    # unrecovered B2[6] escalation/chime state remains suppressed.
    for i in range(1, 10):
      ci.apply(control(1.0, left_lane=True, right_lane=True, steer_alert=True), 3_000_000_000 + i * 10_000_000)
    _, sends = ci.apply(control(1.0, left_lane=True, right_lane=True, steer_alert=True), 3_100_000_000)
    hud = next(dat for addr, dat, bus in sends if addr == 0x412 and bus == 0)
    self.assertEqual(hud, bytes.fromhex("140c004401ee9307"))

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

  def test_b6_sequence_progresses_while_inline_signer_trailer_stays_zero(self):
    ci = CarInterface(self.CP)
    update_with_frame_set(ci, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    _, sends = ci.apply(control(1.0), 2_000_000_000)
    first = next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0)
    ci.apply(control(1.0), 2_010_000_000)
    _, sends = ci.apply(control(1.0), 2_020_000_000)
    second = next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0)

    self.assertEqual((second[7] - first[7]) & 0x3F, 1)
    self.assertEqual(first[28:32], bytes(4))
    self.assertEqual(second[28:32], bytes(4))

  def test_b6_inline_signer_marker_is_independent_of_host_sync_epoch(self):
    ci = CarInterface(self.CP)
    base = CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]}
    update_with_frame_set(ci, base)
    _, sends = ci.apply(control(1.0), 2_000_000_000)
    first = next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0)

    sync = bytearray(CAMRY_COMMON[0x00F])
    reset = (sync[2] << 12) | (sync[3] << 4) | (sync[4] >> 4)
    reset = (reset + 1) & 0xFFFFF
    sync[2] = (reset >> 12) & 0xFF
    sync[3] = (reset >> 4) & 0xFF
    sync[4] = (sync[4] & 0x0F) | ((reset & 0x0F) << 4)
    update_with_frame_set(ci, base | {0x00F: bytes(sync)}, repeats=1)
    ci.apply(control(1.0), 2_010_000_000)
    _, sends = ci.apply(control(1.0), 2_020_000_000)
    after_reset = next(dat for addr, dat, bus in sends if addr == 0x0B6 and bus == 0)

    self.assertEqual(first[28:32], bytes(4))
    self.assertEqual(after_reset[28:32], bytes(4))
    self.assertEqual((after_reset[7] - first[7]) & 0x3F, 1)

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

  def test_brake_cancel_safety_checks_cancel_bit_checksum_and_bus(self):
    good = bytes.fromhex("8800000600000098")
    self.assertTrue(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, good)))

    brake_off = bytearray(good)
    brake_off[0] &= ~0x08
    brake_off[7] = (8 + 1 + 1 + sum(brake_off[:7])) & 0xFF
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, bytes(brake_off))))

    # Unrelated Brake Module status bytes are stock-cloned by CarController and
    # are not a second Panda permission/template surface.
    varied_stock = bytearray(good)
    varied_stock[4] = 1
    varied_stock[7] = (8 + 1 + 1 + sum(varied_stock[:7])) & 0xFF
    self.assertTrue(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, bytes(varied_stock))))

    bad_checksum = bytearray(good)
    bad_checksum[7] ^= 1
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 2, bytes(bad_checksum))))

    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x101, 0, good)))

  def test_relay_forwards_stock_08a_and_replaces_camera_owned_messages(self):
    self.assertEqual(self.s.safety_fwd_hook(2, 0x08A), 0)
    self.assertEqual(self.s.safety_fwd_hook(2, 0x0B6), -1)
    self.assertEqual(self.s.safety_fwd_hook(2, 0x412), -1)

    hud = bytes.fromhex("1400004401ee9307")
    self.assertTrue(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x412, 0, hud)))
    self.assertFalse(self.s.safety_tx_hook(libsafety_py.make_CANPacket(0x412, 2, hud)))


if __name__ == "__main__":
  unittest.main()
