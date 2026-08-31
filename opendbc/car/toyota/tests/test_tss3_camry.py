import unittest

from Crypto.Cipher import AES
from Crypto.Hash import CMAC

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, CanData, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.fw_query_definitions import StdQueries
from opendbc.car.fw_versions import build_fw_dict, match_fw_to_car
from opendbc.car.toyota.fingerprints import FINGERPRINTS, FW_VERSIONS, TSS3_CAN_CENSUS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.tss3 import (
  InstrumentedTSS3Signer,
  TSS3B6CompanionFields,
  TSS3B6Template,
  TSS3Freshness,
  TSS3Gate2DevelopmentConfig,
  TSS3Gate2DevelopmentSender,
  TSS3PandaSafetyCandidate,
  TSS3ReplacementFreshnessState,
  TSS3SignerResult,
  TSS3_B6_AUTH_INPUT_LEN,
  TSS3_B6_TARGET_ANGLE_MAX_RAW,
  TSS3_B6_TARGET_DELTA_MAX_PER_GAP_RAW,
  TSS3_B6_TARGET_ANGLE_SCALE_DEG,
  decode_eps_394_state_candidates,
  build_b6_application,
  build_b6_auth_input,
  build_b6_trailer,
  sign_b6,
  target_angle_deg_to_raw,
)
from opendbc.car.toyota.values import CAR, DBC, FW_QUERY_CONFIG, TSS3_EXACT_FW_VERSIONS, ToyotaFlags, ToyotaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py

Ecu = structs.CarParams.Ecu

# Verbatim retained same-car bus-1 frames. The gear payloads come from the
# identity-bound NRTD->READY selector captures; the remaining state frames are
# representative values from the same capture.
CAMRY_COMMON = {
  0x00F: bytes.fromhex("01b20145cde4b47d"),
  0x025: bytes.fromhex("000100005000007e0000000000000000000000000000000000000000bb6fee54"),
  0x08A: bytes.fromhex("0000000880002d47fe462afe467fff007fffff35c00b100064003c005db7797f"),
  0x030: bytes.fromhex("00000000170001500000100026820000000000010000ffff00000000b280595f"),
  0x0AA: bytes.fromhex("1a6f1a6f1a6f1a6f"),
  0x101: bytes.fromhex("800000010000008b"),
  0x116: bytes.fromhex("000000007b4b235a"),
  0x176: bytes.fromhex("8800000000000007"),
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


def fingerprint_on(bus: int) -> dict[int, dict[int, int]]:
  fp = {i: {} for i in range(8)}
  fp[bus] = {0x025: 32, 0x0AA: 8}
  return fp


def update_with_frame_set(ci: CarInterface, frames: dict[int, bytes], repeats: int = 20, bus: int = 1):
  # Exact Camry relay-open topology: 0x08A originates on camera-side bus 2;
  # vehicle/EPS state remains on the PT side selected by the test fingerprint.
  packet = [CanData(address, dat, 2 if address == 0x08A else bus) for address, dat in frames.items()]
  ret = None
  for i in range(repeats):
    ret = ci.update([(1_000_000_000 + i * 10_000_000, packet)])
  return ret


class _AesCmacSigner:
  def __init__(self, key: bytes):
    self.key = key

  def sign_cmac128(self, auth_input: bytes) -> TSS3SignerResult:
    cmac = CMAC.new(self.key, ciphermod=AES)
    cmac.update(auth_input)
    return TSS3SignerResult(cmac.digest())


class _ExpectedSyncAuthenticator:
  def __init__(self, trip_counter: int, reset_counter: int, authenticator: int):
    self.expected = (trip_counter, reset_counter, authenticator)

  def verify_sync(self, *, trip_counter: int, reset_counter: int, authenticator: int) -> bool:
    return (trip_counter, reset_counter, authenticator) == self.expected


class _FixedSigner:
  def sign_cmac128(self, auth_input: bytes) -> TSS3SignerResult:
    assert len(auth_input) == TSS3_B6_AUTH_INPUT_LEN
    return TSS3SignerResult(bytes.fromhex("123456789abcdef00112233445566778"))


class TestToyotaCamryTSS3Platform(unittest.TestCase):
  def test_exact_platform_is_strictly_passive(self):
    CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint_on(1), [], False, False, False)
    self.assertTrue(CP.flags & ToyotaFlags.TSS3)
    self.assertTrue(CP.flags & ToyotaFlags.SECOC)
    self.assertTrue(CP.flags & ToyotaFlags.TSS3_PT_BUS1)
    self.assertFalse(CP.flags & ToyotaFlags.TSS2)
    self.assertEqual(DBC[CAR.TOYOTA_CAMRY_TSS3][Bus.pt], "toyota_tss3_pt_generated")
    self.assertTrue(CP.dashcamOnly)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)
    self.assertTrue(CP.secOcRequired)
    self.assertEqual(CP.steerControlType, structs.CarParams.SteerControlType.angle)
    self.assertFalse(CP.openpilotLongitudinalControl)

  def test_exact_identity_binding_is_partial_and_route_explicit(self):
    fw = TSS3_EXACT_FW_VERSIONS[CAR.TOYOTA_CAMRY_TSS3]
    self.assertEqual(fw[(Ecu.eps, 0x7A1, None)], [
      bytes.fromhex("023839363546333330373030300000000038413331313333303331303000000000")])
    self.assertEqual(fw[(Ecu.fwdCamera, 0x792, None)], [bytes.fromhex("0138363436463333313530303000000000")])
    self.assertEqual(fw[(Ecu.abs, 0x7B0, None)], [bytes.fromhex("01463135323633334b3030303000000000")])
    fw_db = __import__("opendbc.car.toyota.fingerprints", fromlist=["FW_VERSIONS"]).FW_VERSIONS
    self.assertEqual(fw_db[CAR.TOYOTA_CAMRY_TSS3], fw)

  def test_exact_stationary_can_census_contains_f33_network_but_is_not_legacy_fingerprint(self):
    fp = TSS3_CAN_CENSUS[CAR.TOYOTA_CAMRY_TSS3]
    for address, size in {0x00F: 8, 0x025: 32, 0x030: 32, 0x0D7: 32, 0x127: 8, 0x51E: 8}.items():
      self.assertEqual(fp[address], size)
    self.assertEqual(FINGERPRINTS[CAR.TOYOTA_CAMRY_TSS3][0], fp)

  def test_relay_correct_toyota_fw_query_already_requests_eps_f181_on_bus0(self):
    uds_f181 = [r for r in FW_QUERY_CONFIG.requests if r.bus == 0 and Ecu.eps in r.whitelist_ecus and
                StdQueries.UDS_VERSION_REQUEST in r.request]
    self.assertGreaterEqual(len(uds_f181), 2)
    direct = [r for r in uds_f181 if r.request == [StdQueries.UDS_VERSION_REQUEST]]
    self.assertEqual(len(direct), 1)
    self.assertEqual(StdQueries.UDS_VERSION_REQUEST, b"\x22\xf1\x81")

  def test_exact_eps_f181_binds_camry_without_ambiguous_can_fingerprint(self):
    exact = TSS3_EXACT_FW_VERSIONS[CAR.TOYOTA_CAMRY_TSS3]
    car_fw = []
    for (ecu, address, sub_address), versions in exact.items():
      car_fw.append(structs.CarParams.CarFw(ecu=ecu, address=address, subAddress=0 if sub_address is None else sub_address,
                                            fwVersion=versions[0], brand="toyota"))
    live = build_fw_dict(car_fw)
    self.assertEqual(FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live, "", FW_VERSIONS), {str(CAR.TOYOTA_CAMRY_TSS3)})
    exact_match, matches = match_fw_to_car(car_fw, "", log=False)
    # Once registered in FW_VERSIONS, the standard exact matcher owns this
    # platform and the exact EPS F181 is the required discriminator.
    self.assertTrue(exact_match)
    self.assertEqual(matches, {str(CAR.TOYOTA_CAMRY_TSS3)})

    # Startup discovery only needs the exact EPS identity; camera/ABS are
    # corroborating non-essential ECUs for this maintainer platform.
    eps_ecu = next(ecu for ecu in exact if ecu[0] == Ecu.eps)
    eps_only = [structs.CarParams.CarFw(ecu=eps_ecu[0], address=eps_ecu[1], subAddress=0,
                                        fwVersion=exact[eps_ecu][0], brand="toyota")]
    eps_exact, eps_matches = match_fw_to_car(eps_only, "", log=False)
    self.assertTrue(eps_exact)
    self.assertEqual(eps_matches, {str(CAR.TOYOTA_CAMRY_TSS3)})

    eps_addr = eps_ecu[1:]
    wrong_eps = dict(live)
    wrong_eps[eps_addr] = {b"\x028965F3307001\x00\x00\x00\x008A3113303100\x00\x00\x00\x00"}
    self.assertNotIn(str(CAR.TOYOTA_CAMRY_TSS3), FW_QUERY_CONFIG.match_fw_to_car_fuzzy(wrong_eps, "", FW_VERSIONS))

    frc_addr = next(ecu[1:] for ecu in exact if ecu[0] == Ecu.fwdCamera)
    conflicting = dict(live)
    conflicting[frc_addr] = {b"\x018646F3315999\x00\x00\x00\x00"}
    self.assertNotIn(str(CAR.TOYOTA_CAMRY_TSS3), FW_QUERY_CONFIG.match_fw_to_car_fuzzy(conflicting, "", FW_VERSIONS))

  def test_source_real_camry_full_gear_enum(self):
    for expected, gear_frame in CAMRY_GEAR.items():
      with self.subTest(expected=expected):
        CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint_on(1), [], False, False, False)
        CI = CarInterface(CP)
        CS = update_with_frame_set(CI, CAMRY_COMMON | {0x127: gear_frame})
        self.assertTrue(CS.canValid)
        self.assertEqual(CS.gearShifter, expected)

  def test_source_real_ready_zero_and_one_are_observable_without_fault_policy(self):
    for ready, payload in [(0, bytes.fromhex("0000640000000000")), (1, bytes.fromhex("80006e0000000000"))]:
      CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint_on(1), [], False, False, False)
      CI = CarInterface(CP)
      CS = update_with_frame_set(CI, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.park], 0x51E: payload})
      self.assertEqual(CI.CS.tss3_ready_status, bool(ready))
      self.assertFalse(CS.steerFaultTemporary)
      self.assertFalse(CS.steerFaultPermanent)

  def test_source_real_lateral_request_is_observable_only(self):
    CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint_on(1), [], False, False, False)
    CI = CarInterface(CP)
    CS = update_with_frame_set(CI, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    self.assertTrue(CI.CS.tss3_lateral_request_seen)
    self.assertEqual(CI.CS.tss3_target_lateral_id, 11)
    self.assertAlmostEqual(CI.CS.tss3_lateral_request_angle, -203 * TSS3_B6_TARGET_ANGLE_SCALE_DEG)
    self.assertEqual(CI.CS.tss3_lateral_request_sequence, 60)
    self.assertAlmostEqual(CI.CS.tss3_steering_assist_gain, 1.0)
    self.assertTrue(CP.dashcamOnly)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)
    self.assertTrue(CS.cruiseState.enabled)
    self.assertTrue(CS.cruiseState.available)
    self.assertAlmostEqual(CS.cruiseState.speed, 42 * CV.KPH_TO_MS, places=5)

  def test_camry_cruise_latch_off_clears_enabled(self):
    CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint_on(1), [], False, False, False)
    CI = CarInterface(CP)
    off = bytearray(CAMRY_COMMON[0x08A])
    off[3] = 0
    off[10] = 0
    CS = update_with_frame_set(CI, CAMRY_COMMON | {
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
      0x08A: bytes(off),
    })
    self.assertFalse(CS.cruiseState.enabled)
    self.assertFalse(CS.cruiseState.available)
    self.assertEqual(CS.cruiseState.speed, 0.0)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)

  def test_static_f33_status_carriers_are_presence_bounded(self):
    CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint_on(1), [], False, False, False)
    CI = CarInterface(CP)
    CS = update_with_frame_set(CI, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.park]})
    self.assertFalse(CI.CS.tss3_alt_telemetry_seen)
    self.assertFalse(CI.CS.tss3_status_351_seen)
    self.assertFalse(CI.CS.tss3_fault_394_seen)
    self.assertFalse(CS.steerFaultTemporary)
    self.assertFalse(CS.steerFaultPermanent)

    packer = CANPacker("toyota_tss3_pt_generated")
    synthetic_static_projection = {
      0x4A3: packer.make_can_msg("TSS3_ALT_STEERING_TELEMETRY", 1, {
        "MARKER_BIT": 1, "STEERING_FAULT_INHIBIT_STATUS": 1,
        "STEERING_WHEEL_TORQUE": 1.2, "MOTOR_CURRENT_ALT_RAW": -234,
      })[1],
      0x351: packer.make_can_msg("TSS3_EPS_STATUS_351", 1, {"STATUS_CODE": 7, "STATUS_FLAG": 1})[1],
      0x394: packer.make_can_msg("TSS3_EPS_FAULT_STATUS_394", 1, {
        "STATUS_TABLE_COLUMN_4": 2, "STATUS_TABLE_COLUMN_1": 3,
        "STATUS_TABLE_COLUMN_2": 4, "STATUS_TABLE_COLUMN_3": 1,
      })[1],
    }
    update_with_frame_set(CI, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.park]} | synthetic_static_projection)
    self.assertTrue(CI.CS.tss3_alt_telemetry_seen)
    self.assertAlmostEqual(CI.CS.tss3_alt_steering_torque, 1.2)
    self.assertEqual(CI.CS.tss3_motor_current_alt_raw, -234)
    self.assertTrue(CI.CS.tss3_status_351_seen)
    self.assertEqual((CI.CS.tss3_status_351_code, CI.CS.tss3_status_351_flag), (7, True))
    self.assertTrue(CI.CS.tss3_fault_394_seen)
    self.assertEqual(CI.CS.tss3_fault_394_projection, (2, 3, 4, 1))
    self.assertEqual(CI.CS.tss3_fault_394_state_candidates, ())
    self.assertIsNone(CI.CS.tss3_fault_394_state)
    self.assertFalse(CS.steerFaultTemporary)
    self.assertFalse(CS.steerFaultPermanent)

  def test_exact_f33_394_projection_decodes_internal_state_candidates_only(self):
    # Target-native F33 state table at CodeFlash 0x2A19C. The four-tuple is
    # (column4, column1, column2, column3), exactly the fields carried by 0x394.
    self.assertEqual(decode_eps_394_state_candidates((0, 0, 0, 0)), (0,))
    self.assertEqual(decode_eps_394_state_candidates((2, 3, 2, 1)), (6,))
    self.assertEqual(decode_eps_394_state_candidates((1, 7, 1, 1)), (10,))
    self.assertEqual(decode_eps_394_state_candidates((0, 3, 0, 0)), (1, 3, 4))
    self.assertEqual(decode_eps_394_state_candidates((0, 7, 0, 0)), (2, 16))
    self.assertEqual(decode_eps_394_state_candidates((3, 7, 7, 1)), ())

    CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint_on(1), [], False, False, False)
    CI = CarInterface(CP)
    packer = CANPacker("toyota_tss3_pt_generated")
    clear_394 = packer.make_can_msg("TSS3_EPS_FAULT_STATUS_394", 1, {
      "STATUS_TABLE_COLUMN_4": 0, "STATUS_TABLE_COLUMN_1": 0,
      "STATUS_TABLE_COLUMN_2": 0, "STATUS_TABLE_COLUMN_3": 0,
    })[1]
    CS = update_with_frame_set(CI, CAMRY_COMMON | {
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.park], 0x394: clear_394,
    })
    self.assertEqual(CI.CS.tss3_fault_394_state_candidates, (0,))
    self.assertEqual(CI.CS.tss3_fault_394_state, 0)
    # Internal state0 is not independently a Ready authorization bit or an
    # openpilot temporary/permanent fault policy.
    self.assertFalse(CS.steerFaultTemporary)
    self.assertFalse(CS.steerFaultPermanent)

  def test_controller_computes_shadow_candidate_but_emits_no_can(self):
    CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint_on(1), [], False, False, False)
    CI = CarInterface(CP)
    update_with_frame_set(CI, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})

    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.actuators.steeringAngleDeg = 2 * TSS3_B6_TARGET_ANGLE_SCALE_DEG
    _, sends = CI.apply(CC.as_reader(), 2_000_000_000)
    self.assertEqual(sends, [])
    self.assertEqual(CI.CC.tss3_last_application.target_lateral_id, 11)
    self.assertEqual(CI.CC.tss3_last_application.target_angle_raw, 2)
    self.assertFalse(CI.CC.tss3_last_application.template_stock_validated)
    self.assertTrue(CI.CC.tss3_last_safety_decision.static_limits_ok)
    self.assertFalse(CI.CC.tss3_last_safety_decision.production_allowed)


class TestToyotaTSS3B6Contract(unittest.TestCase):
  def test_application_packer_preserves_unknown_template_bits(self):
    template_bytes = bytearray(range(28))
    template = TSS3B6Template(bytes(template_bytes), stock_validated=False, provenance="unit-test explicit unknowns")
    companions = TSS3B6CompanionFields(
      signal263_b6_bit7=1, signal264_b6_bits6_4=5, additive_term_suppress=1,
      signal266_b6_bits1_0=2, signal267_b7_bits7_6=2,
      contribution_pct_1=12, contribution_pct_2=34,
      signal271_b10_bit7=1, signal272_b10_bit5=1, signal273_b10_bits2_0=5,
    )
    app = build_b6_application(target_lateral_id=11, target_angle_raw=-123, sequence=63,
                               template=template, companions=companions)
    self.assertEqual(len(app.data), 28)
    self.assertEqual(app.data[0:3], bytes(range(3)))
    self.assertEqual(app.data[3] & 0x3F, 11)
    self.assertEqual(int.from_bytes(app.data[4:6], "big", signed=True), -123)
    self.assertEqual(app.data[6] & 0xF7, 0xD6)
    self.assertEqual(app.data[6] & 0x08, template_bytes[6] & 0x08)
    self.assertEqual(app.data[7], 0xBF)
    self.assertEqual(app.data[8:10], bytes((12, 34)))
    self.assertEqual(app.data[10] & 0xA7, 0xA5)
    self.assertEqual(app.data[11:], bytes(range(11, 28)))

  def test_dbc_b6_scalar_projection_matches_manual_packer(self):
    companions = TSS3B6CompanionFields(additive_term_suppress=1, contribution_pct_1=12, contribution_pct_2=34)
    app = build_b6_application(target_lateral_id=11, target_angle_raw=1, sequence=7,
                               template=TSS3B6Template(), companions=companions)
    packer = CANPacker("toyota_tss3_pt_generated")
    dbc = packer.make_can_msg("TSS3_LATERAL_CONTROL", 0, {
      "TARGET_LATERAL_ID": 11,
      "TARGET_STEERING_ANGLE": TSS3_B6_TARGET_ANGLE_SCALE_DEG,
      "ADDITIVE_TERM_SUPPRESS": 1,
      "SEQUENCE": 7,
      "CONTRIBUTION_PCT_1": 12,
      "CONTRIBUTION_PCT_2": 34,
    })[1]
    self.assertEqual(app.data[:11], dbc[:11])

  def test_replacement_freshness_requires_newer_authenticated_sync(self):
    state = TSS3ReplacementFreshnessState()
    self.assertFalse(state.observe_sync(1, 100, authenticated=False))
    with self.assertRaises(RuntimeError):
      state.next()
    self.assertFalse(state.observe_sync(1, 100, authenticated=True))
    self.assertFalse(state.observe_sync(1, 100, authenticated=True))
    self.assertFalse(state.observe_sync(1, 101, authenticated=False))
    self.assertTrue(state.observe_sync(1, 101, authenticated=True))

    first_fv, first_seq = state.next()
    second_fv, second_seq = state.next()
    self.assertEqual((first_fv.trip_counter, first_fv.reset_counter, first_fv.message_counter, first_seq), (1, 101, 1, 0))
    self.assertEqual((second_fv.message_counter, second_seq), (2, 1))

  def test_source_real_00f_decodes_then_requires_explicit_authenticator(self):
    parser = CANParser("toyota_tss3_pt_generated", [("SECOC_SYNCHRONIZATION", float('nan'))], 1)
    parser.update([(1, [CanData(0x00F, CAMRY_COMMON[0x00F], 1)])])
    sync = dict(parser.vl["SECOC_SYNCHRONIZATION"])
    self.assertEqual((int(sync["TRIP_CNT"]), int(sync["RESET_CNT"]), int(sync["AUTHENTICATOR"])),
                     (434, 5212, 233092221))

    state = TSS3ReplacementFreshnessState()
    bad = _ExpectedSyncAuthenticator(434, 5212, 0)
    good = _ExpectedSyncAuthenticator(434, 5212, 233092221)
    self.assertFalse(state.observe_sync_fields(sync, bad))
    self.assertFalse(state.observe_sync_fields(sync, good))  # authenticated baseline only

  def test_exact_freshness_auth_input_and_trailer_geometry(self):
    freshness = TSS3Freshness(0x1234, 0xABCDE, 0x41)
    app = bytes(range(28))
    auth_input = build_b6_auth_input(app, freshness)
    self.assertEqual(len(auth_input), 36)
    self.assertEqual(auth_input[:2], b"\x00\xb6")
    self.assertEqual(auth_input[2:30], app)
    self.assertEqual(auth_input[30:], freshness.full_bytes())
    self.assertEqual(freshness.transmitted_nibble, 0x6)  # msg low2=1, reset low2=2
    self.assertEqual(build_b6_trailer(freshness, bytes.fromhex("123456789abcdef00112233445566778")), bytes.fromhex("61234567"))

  def test_full_signed_b6_uses_36_byte_cmac_input(self):
    key = bytes(range(16))
    app = build_b6_application(target_lateral_id=11, target_angle_raw=17, sequence=4, template=TSS3B6Template())
    freshness = TSS3Freshness(0x100, 0x12345, 0x22)
    signed = sign_b6(app, freshness, _AesCmacSigner(key))
    self.assertEqual(len(signed.data), 32)
    self.assertEqual(signed.data[:28], app.data)
    cmac = CMAC.new(key, ciphermod=AES)
    cmac.update(signed.auth_input)
    self.assertEqual(signed.data[28:], build_b6_trailer(freshness, cmac.digest()))

  def test_signer_latency_instrumentation(self):
    ticks = iter((1000, 1125, 2000, 2250))
    signer = InstrumentedTSS3Signer(_FixedSigner(), clock_ns=lambda: next(ticks))
    auth = bytes(TSS3_B6_AUTH_INPUT_LEN)
    signer.sign_cmac128(auth)
    signer.sign_cmac128(auth)
    self.assertEqual(signer.stats.calls, 2)
    self.assertEqual(signer.stats.successes, 2)
    self.assertEqual(signer.stats.failures, 0)
    self.assertEqual(signer.stats.min_latency_ns, 125)
    self.assertEqual(signer.stats.max_latency_ns, 250)
    self.assertEqual(signer.stats.mean_latency_ns, 187.5)

  def test_python_safety_candidate_is_strict_and_never_production_authorizes(self):
    safety = TSS3PandaSafetyCandidate()
    ok = safety.check(target_lateral_id=11, target_angle_raw=100, sequence=0,
                      steering_angle_velocity_raw=0, now_nanos=0)
    self.assertTrue(ok.static_limits_ok)
    self.assertFalse(ok.production_allowed)
    ok = safety.check(target_lateral_id=11, target_angle_raw=178, sequence=1,
                      steering_angle_velocity_raw=100, now_nanos=10_000_000)
    self.assertTrue(ok.static_limits_ok)
    self.assertFalse(ok.production_allowed)

    for kwargs in [
      dict(target_lateral_id=4, target_angle_raw=178, sequence=2, steering_angle_velocity_raw=0, now_nanos=20_000_000),
      dict(target_lateral_id=11, target_angle_raw=TSS3_B6_TARGET_ANGLE_MAX_RAW + 1, sequence=2, steering_angle_velocity_raw=0, now_nanos=20_000_000),
      dict(target_lateral_id=11, target_angle_raw=257, sequence=2, steering_angle_velocity_raw=0, now_nanos=20_000_000),
      dict(target_lateral_id=11, target_angle_raw=178, sequence=3, steering_angle_velocity_raw=101, now_nanos=20_000_000),
    ]:
      decision = safety.check(**kwargs)
      self.assertFalse(decision.static_limits_ok)
      self.assertFalse(decision.production_allowed)

  def test_gate2_development_sender_requires_live_supplied_gates(self):
    with self.assertRaises(ValueError):
      TSS3Gate2DevelopmentConfig(TSS3B6Template(bytes(28), stock_validated=False), 1, True, True)
    with self.assertRaises(ValueError):
      TSS3Gate2DevelopmentConfig(TSS3B6Template(bytes(28), stock_validated=True), 1, False, True)
    with self.assertRaises(ValueError):
      TSS3Gate2DevelopmentConfig(TSS3B6Template(bytes(28), stock_validated=True), 4, True, True)

    sender = TSS3Gate2DevelopmentSender(TSS3Gate2DevelopmentConfig(
      TSS3B6Template(bytes(range(28)), stock_validated=True), 2, True, True,
    ))
    baseline = {"TRIP_CNT": 1, "RESET_CNT": 100, "AUTHENTICATOR": 0}
    newer = {"TRIP_CNT": 1, "RESET_CNT": 101, "AUTHENTICATOR": 0}
    self.assertFalse(sender.observe_sync_unverified_for_gate2_development(baseline))
    self.assertTrue(sender.observe_sync_unverified_for_gate2_development(newer))
    self.assertIsNone(sender.build_if_due(frame=1, desired_target_raw=1000, steering_angle_velocity_raw=0, now_nanos=1_000_000))
    out = sender.build_if_due(frame=2, desired_target_raw=1000, steering_angle_velocity_raw=0, now_nanos=2_000_000)
    self.assertIsNotNone(out)
    self.assertEqual(out.application.target_angle_raw, 1000)
    self.assertEqual(int.from_bytes(out.data[28:], "big") & 0x0FFFFFFF, 0)
    # The next command is clamped to the exact-F33 +78 raw replacement step.
    out2 = sender.build_if_due(frame=4, desired_target_raw=1500, steering_angle_velocity_raw=0, now_nanos=12_000_000)
    self.assertIsNotNone(out2)
    self.assertEqual(out2.application.target_angle_raw, 1078)
    sender.note_inactive()
    self.assertIsNone(sender.build_if_due(frame=6, desired_target_raw=1000, steering_angle_velocity_raw=0, now_nanos=22_000_000))


class TestToyotaTSS3PandaShadowSafety(unittest.TestCase):
  def test_c_candidate_limits(self):
    s = libsafety_py.libsafety
    check = s.test_toyota_tss3_candidate_limits_check
    self.assertTrue(check(11, 100, 0, 0, 0, 0, 0, False))
    self.assertTrue(check(11, 178, 1, 100, 0, 100, 10_000, True))
    self.assertFalse(check(4, 100, 0, 0, 0, 0, 0, False))
    self.assertFalse(check(11, 1746, 0, 0, 0, 0, 0, False))
    self.assertFalse(check(11, 179, 1, 100, 0, 0, 10_000, True))
    self.assertFalse(check(11, 100, 2, 100, 0, 0, 10_000, True))
    self.assertFalse(check(11, 100, 1, 100, 0, 101, 10_000, True))
    self.assertFalse(check(11, 100, 1, 100, 0, 0, 35_001, True))
    self.assertTrue(check(0, 0, 1, 0, 0, 0, 99_999, True))
    self.assertFalse(check(0, 1, 1, 0, 0, 0, 10_000, True))

  def test_actual_panda_mode_remains_no_output_and_rejects_b6(self):
    s = libsafety_py.libsafety
    self.assertEqual(s.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0), 0)
    s.init_tests()
    candidate = libsafety_py.make_CANPacket(0x0B6, 0, bytes(32))
    self.assertFalse(s.safety_tx_hook(candidate))

  def test_debug_panda_requires_cruise_engagement_for_active_b6(self):
    s = libsafety_py.libsafety
    self.assertEqual(s.set_safety_hooks(structs.CarParams.SafetyModel.toyota, ToyotaSafetyFlags.TSS3_DEV_LATERAL), 0)
    s.init_tests()

    def b6(angle: int, sequence: int, target_id: int):
      dat = bytearray(32)
      dat[3] = target_id
      dat[4:6] = angle.to_bytes(2, "big", signed=True)
      dat[6] = 0x00 if target_id != 0 else 0x04
      dat[7] = sequence
      if target_id != 0:
        dat[8] = 100
        dat[9] = 100
      return libsafety_py.make_CANPacket(0x0B6, 0, bytes(dat))

    # Prime steering-rate and stock sync inputs, then the Camry cruise
    # latch on 0x08A B3[3] — the dev mode's engagement source.
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x025, 0, CAMRY_COMMON[0x025])))
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x00F, 0, CAMRY_COMMON[0x00F])))
    stock_lateral_off = bytearray(CAMRY_COMMON[0x08A])
    stock_lateral_off[21] = 0
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, bytes(stock_lateral_off))))
    self.assertTrue(s.get_controls_allowed())

    s.set_controls_allowed(False)
    self.assertFalse(s.safety_tx_hook(b6(100, 0, 11)))
    self.assertTrue(s.safety_tx_hook(b6(0, 0, 0)))
    s.set_controls_allowed(True)
    self.assertTrue(s.safety_tx_hook(b6(50, 1, 11)))

  def test_debug_panda_tracks_camry_cruise_latch(self):
    s = libsafety_py.libsafety
    self.assertEqual(s.set_safety_hooks(structs.CarParams.SafetyModel.toyota, ToyotaSafetyFlags.TSS3_DEV_LATERAL), 0)
    s.init_tests()
    cruise_on = bytearray(CAMRY_COMMON[0x08A])
    cruise_off = bytearray(cruise_on)
    cruise_off[3] &= ~(1 << 3)

    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, bytes(cruise_on))))
    self.assertTrue(s.get_controls_allowed())
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, bytes(cruise_off))))
    self.assertFalse(s.get_controls_allowed())

  def test_debug_panda_inactive_release_reanchors_after_blocked_active(self):
    s = libsafety_py.libsafety
    self.assertEqual(s.set_safety_hooks(structs.CarParams.SafetyModel.toyota, ToyotaSafetyFlags.TSS3_DEV_LATERAL), 0)
    s.init_tests()

    def b6(angle: int, sequence: int, target_id: int):
      dat = bytearray(32)
      dat[3] = target_id
      dat[4:6] = angle.to_bytes(2, "big", signed=True)
      dat[6] = 0x00 if target_id else 0x04
      dat[7] = sequence
      if target_id:
        dat[8:10] = b"\x64\x64"
      return libsafety_py.make_CANPacket(0x0B6, 0, bytes(dat))

    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x025, 0, CAMRY_COMMON[0x025])))
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x00F, 0, CAMRY_COMMON[0x00F])))
    stock_off = bytearray(CAMRY_COMMON[0x08A])
    stock_off[21] = 0
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, bytes(stock_off))))
    self.assertTrue(s.safety_tx_hook(b6(100, 0, 11)))

    # Stock lateral wins before the next active frame: Panda blocks it.
    stock_on = bytearray(stock_off)
    stock_on[21] = 11
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, bytes(stock_on))))
    self.assertFalse(s.safety_tx_hook(b6(150, 1, 11)))

    # The sender has already advanced to sequence 2. A non-actuating release is
    # allowed to re-anchor there, then the next active command must be +1.
    self.assertTrue(s.safety_tx_hook(b6(0, 2, 0)))
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, bytes(stock_off))))
    self.assertTrue(s.safety_tx_hook(b6(50, 3, 11)))

  def test_debug_panda_path_is_b6_only_and_enforces_f33_limits(self):
    s = libsafety_py.libsafety
    self.assertEqual(s.set_safety_hooks(structs.CarParams.SafetyModel.toyota, ToyotaSafetyFlags.TSS3_DEV_LATERAL), 0)
    s.init_tests()

    def b6(angle: int, sequence: int, target_id: int = 11):
      dat = bytearray(32)
      dat[3] = target_id
      dat[4:6] = angle.to_bytes(2, "big", signed=True)
      dat[6] = 0x00 if target_id != 0 else 0x04
      dat[7] = sequence
      if target_id != 0:
        dat[8] = 100
        dat[9] = 100
      return libsafety_py.make_CANPacket(0x0B6, 0, bytes(dat))

    # No command before both target-native steering-rate and stock sync inputs.
    self.assertFalse(s.safety_tx_hook(b6(100, 0)))
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x025, 0, CAMRY_COMMON[0x025])))
    self.assertFalse(s.safety_tx_hook(b6(100, 0)))
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x00F, 0, CAMRY_COMMON[0x00F])))
    stock_lateral_off = bytearray(CAMRY_COMMON[0x08A])
    stock_lateral_off[21] = 0
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, bytes(stock_lateral_off))))
    self.assertTrue(s.safety_tx_hook(b6(100, 0)))
    # An older epoch must not reset the Panda sequence baseline.
    stale_sync = bytearray(CAMRY_COMMON[0x00F])
    stale_sync[3] -= 0x10
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x00F, 0, bytes(stale_sync))))
    self.assertFalse(s.safety_tx_hook(b6(178, 0)))
    s.set_timer(10_000)
    self.assertTrue(s.safety_tx_hook(b6(178, 1)))
    s.set_timer(20_000)
    self.assertFalse(s.safety_tx_hook(b6(257, 2)))  # >78 raw step
    self.assertFalse(s.safety_tx_hook(b6(178, 3, target_id=4)))
    stock_lta_on = bytearray(stock_lateral_off)
    stock_lta_on[21] = 11
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, bytes(stock_lta_on))))
    s.set_timer(30_000)
    self.assertFalse(s.safety_tx_hook(b6(178, 2)))
    self.assertFalse(s.safety_tx_hook(libsafety_py.make_CANPacket(0x191, 0, bytes(8))))
    self.assertFalse(s.safety_tx_hook(libsafety_py.make_CANPacket(0x0B6, 1, bytes(32))))

    # A strictly changed stock sync epoch resets sequence history; a high-rate
    # exact-F33 0x025 input still blocks actuation independently.
    newer_sync = bytearray(CAMRY_COMMON[0x00F])
    newer_sync[3] += 0x10
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x00F, 0, bytes(newer_sync))))
    high_rate = bytearray(CAMRY_COMMON[0x025])
    high_rate[4] = (high_rate[4] & 0xF0) | 0x0
    high_rate[5] = 101
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x025, 0, bytes(high_rate))))
    self.assertFalse(s.safety_tx_hook(b6(100, 0)))


class TestToyotaTSS3BridgeSender(unittest.TestCase):
  def _bridge_platform(self) -> CarInterface:
    CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint_on(1), [], False, False, False)
    CI = CarInterface(CP)
    stock_lateral_off = bytearray(CAMRY_COMMON[0x08A])
    stock_lateral_off[21] = 0
    update_with_frame_set(CI, CAMRY_COMMON | {
      0x08A: bytes(stock_lateral_off),
      0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive],
    })
    CI.CC.ephemeral_secoc_bridge = True
    return CI

  @staticmethod
  def _control(angle_deg: float, lat_active: bool = True):
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = lat_active
    CC.actuators.steeringAngleDeg = angle_deg
    return CC.as_reader()

  def test_bridge_reports_slew_limited_angle(self):
    CI = self._bridge_platform()
    _, first_sends = CI.apply(self._control(1.0), 2_000_000_000)
    first_raw = int.from_bytes(first_sends[0][1][4:6], "big", signed=True)
    # Same sync epoch: a large jump is clamped to the per-gap slew envelope.
    actuators, sends = CI.apply(self._control(20.0), 2_000_010_000)
    sent_raw = int.from_bytes(sends[0][1][4:6], "big", signed=True)
    self.assertEqual(sent_raw, first_raw + TSS3_B6_TARGET_DELTA_MAX_PER_GAP_RAW)
    self.assertAlmostEqual(actuators.steeringAngleDeg, sent_raw * TSS3_B6_TARGET_ANGLE_SCALE_DEG, places=4)

  def test_bridge_frame_shape_matches_f33_receiver_contract(self):
    CI = self._bridge_platform()
    _, sends = CI.apply(self._control(2 * TSS3_B6_TARGET_ANGLE_SCALE_DEG), 2_000_000_000)
    self.assertEqual(len(sends), 1)
    addr, dat, bus = sends[0]
    self.assertEqual((addr, bus, len(dat)), (0x0B6, 0, 32))
    # Panda hook field projection: B3[5:0], B4:B5 signed BE, B7[5:0].
    self.assertEqual(dat[3] & 0x3F, 11)
    self.assertEqual(int.from_bytes(dat[4:6], "big", signed=True), 2)
    self.assertEqual(dat[7] & 0x3F, 0)
    self.assertEqual(dat[6], 0x00)  # signal265 clear: do not suppress target-derived contribution
    self.assertEqual(dat[8:10], b"\x64\x64")
    # Zero-MAC28 marker: all 28 MAC bits (B28[3:0], B29, B30, B31) zero,
    # FV4 nibble preserved (fixture: msg low2=0, reset low2=5212&3=0).
    self.assertEqual(dat[28], 0x00)
    self.assertEqual(dat[29:32], b"\x00\x00\x00")

  def test_bridge_sender_sequence_slew_and_real_panda_acceptance(self):
    s = libsafety_py.libsafety
    self.assertEqual(s.set_safety_hooks(structs.CarParams.SafetyModel.toyota, ToyotaSafetyFlags.TSS3_DEV_LATERAL), 0)
    s.init_tests()
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x025, 0, CAMRY_COMMON[0x025])))
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x00F, 0, CAMRY_COMMON[0x00F])))
    stock_lateral_off = bytearray(CAMRY_COMMON[0x08A])
    stock_lateral_off[21] = 0
    self.assertTrue(s.safety_rx_hook(libsafety_py.make_CANPacket(0x08A, 2, bytes(stock_lateral_off))))

    CI = self._bridge_platform()

    def send(angle_deg: float, lat_active: bool = True) -> bytes:
      _, sends = CI.apply(self._control(angle_deg, lat_active), 2_000_000_000)
      self.assertEqual(len(sends), 1)
      addr, dat, bus = sends[0]
      self.assertEqual((addr, bus), (0x0B6, 0))
      self.assertTrue(s.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)))
      return dat

    first = send(10.0)
    self.assertEqual(int.from_bytes(first[4:6], "big", signed=True), target_angle_deg_to_raw(10.0))
    self.assertEqual(first[7] & 0x3F, 0)

    s.set_timer(10_000)
    second = send(20.0)  # large jump: sender pre-clamps to the +/-78 per-gap envelope
    prev_raw = int.from_bytes(first[4:6], "big", signed=True)
    self.assertEqual(int.from_bytes(second[4:6], "big", signed=True), prev_raw + TSS3_B6_TARGET_DELTA_MAX_PER_GAP_RAW)
    self.assertEqual(second[7] & 0x3F, 1)

    # Deactivation is an immediate non-actuating ID0/angle0 release. It is
    # deliberately exempt from the active target slew rule and re-anchors Panda
    # sequence state if the immediately preceding active frame was blocked.
    s.set_timer(20_000)
    dat = send(0.0, lat_active=False)
    self.assertEqual(dat[3] & 0x3F, 0)
    self.assertEqual(int.from_bytes(dat[4:6], "big", signed=True), 0)
    self.assertEqual(dat[6], 0x04)
    self.assertEqual(dat[7] & 0x3F, 2)
    self.assertEqual(dat[8:10], b"\x00\x00")

  def test_bridge_sender_reanchors_on_new_sync_epoch(self):
    CI = self._bridge_platform()
    CI.apply(self._control(1.0), 2_000_000_000)
    CI.apply(self._control(1.0), 2_000_010_000)
    self.assertEqual(CI.CC.tss3_bridge_sequence, 2)
    CI.CS.secoc_synchronization["RESET_CNT"] += 1
    _, sends = CI.apply(self._control(1.0), 2_000_020_000)
    self.assertEqual(sends[0][1][7] & 0x3F, 0)

  def test_bridge_off_still_emits_no_can(self):
    CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint_on(1), [], False, False, False)
    CI = CarInterface(CP)
    update_with_frame_set(CI, CAMRY_COMMON | {0x127: CAMRY_GEAR[structs.CarState.GearShifter.drive]})
    _, sends = CI.apply(self._control(1.0), 2_000_000_000)
    self.assertEqual(sends, [])


if __name__ == "__main__":
  unittest.main()
