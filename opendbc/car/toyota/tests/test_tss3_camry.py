import unittest
import numpy as np
from unittest.mock import patch

from opendbc.car import Bus, CanData, structs
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.tss3 import LTA_LCA_REQUEST_ID, TSS3_CHASSIS_BUS, TSS3_SOURCE_BUS, target_angle_deg_to_raw
from opendbc.car.toyota.values import CAR, CarControllerParams, ToyotaSafetyFlags

CAMRY_COMMON = {
  0x025: bytes.fromhex("000100005000007e0000000000000000000000000000000000000000bb6fee54"),
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
CAMRY_HUD = bytes.fromhex("140c404401ee9307")


def relay_fingerprint() -> dict[int, dict[int, int]]:
  fp = {i: {} for i in range(8)}
  source_ids = {0x08A, 0x251, 0x3F6, 0x412}
  for address, data in (CAMRY_COMMON | {0x412: CAMRY_HUD}).items():
    fp[TSS3_SOURCE_BUS if address in source_ids else TSS3_CHASSIS_BUS][address] = len(data)
  return fp


def update_state(ci: CarInterface, moving: bool = False, counter_offset: int = 0, hud: bytes | None = None,
                 eps_status: int | None = None, eps_telemetry: bytes | None = None,
                 control_request: bytes | None = None, bus: int = TSS3_CHASSIS_BUS, source_bus: int | None = TSS3_SOURCE_BUS,
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
            cancel: bool = False, left_lane: bool = False, right_lane: bool = False, steer_alert: bool = False):
  cc = structs.CarControl()
  cc.enabled = enabled
  cc.latActive = enabled and active
  cc.longActive = enabled and long_active
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
    self.CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)

  def test_alpha_long_gates_longitudinal_ownership(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], False, False, False)
    self.assertTrue(cp.alphaLongitudinalAvailable)
    self.assertFalse(cp.openpilotLongitudinalControl)
    self.assertFalse(cp.autoResumeSng)
    self.assertTrue(cp.pcmCruise)
    self.assertTrue(cp.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL)

    ci = CarInterface(cp)
    update_state(ci, moving=True, bus=0, source_bus=2, hud=CAMRY_HUD)
    output, _ = ci.apply(control(0.0, active=False, accel=1.0, long_active=True), 2_000_000_000)
    self.assertEqual(output.accel, 0.0)

  def test_canonical_tss3_state_uses_chassis_and_source_buses(self):
    ci = CarInterface(self.CP)
    self.assertEqual(ci.can_parsers[Bus.pt].bus, TSS3_CHASSIS_BUS)
    self.assertEqual(ci.can_parsers[Bus.cam].bus, TSS3_SOURCE_BUS)
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
    # driver override with COOPERATIVE_COMMAND_INHIBIT set
    override = bytes.fromhex("12000003330930b9130330053c800e99030b0000053c07b50000000042c3b381")
    state = update_state(CarInterface(self.CP), moving=True, eps_telemetry=override, hud=CAMRY_HUD)
    self.assertTrue(state.canValid)
    self.assertTrue(state.steeringPressed)
    self.assertFalse(state.steerFaultTemporary)
    self.assertFalse(state.steerFaultPermanent)

  def test_reference_initializing_source_does_not_report_a_steering_fault(self):
    # COOPERATIVE_ANGLE_INHIBIT is set while initializing
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

  def test_host_request_plane_uses_normal_angle_control_without_emitting_c7(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)
    ci = CarInterface(cp)
    request = bytearray(CAMRY_COMMON[0x08A])
    request[21] = (request[21] & 0xC0) | 11
    state = update_state(ci, moving=True, control_request=bytes(request), bus=0, source_bus=2, hud=CAMRY_HUD)
    self.assertTrue(state.canValid)

    measured = state.steeringAngleDeg + state.steeringAngleOffsetDeg
    max_delta = CarControllerParams.TSS3_ANGLE_LIMITS.MAX_ANGLE_RATE * ci.CC.params.STEER_STEP
    output, sends = ci.apply(control(0.0, active=False, enabled=False), 2_000_000_000)
    self.assertAlmostEqual(output.steeringAngleDeg, measured, delta=0.01)

    output, sends = ci.apply(control(5.0), 2_010_000_000)
    self.assertFalse(any(address == 0x777 and data[1] == 0xC7 for address, data, _ in sends))
    self.assertEqual(sum(address == 0x777 and 8 <= (data[0] >> 4) <= 0xB for address, data, _ in sends), 4)
    self.assertGreater(output.steeringAngleDeg, measured)
    self.assertLessEqual(output.steeringAngleDeg, measured + max_delta + 1e-6)

    previous_angle = output.steeringAngleDeg
    output, sends = ci.apply(control(20.0), 2_020_000_000)
    self.assertEqual(sum(address == 0x777 and 8 <= (data[0] >> 4) <= 0xB for address, data, _ in sends), 4)
    self.assertGreater(output.steeringAngleDeg, previous_angle)
    self.assertLessEqual(output.steeringAngleDeg, previous_angle + max_delta + 1e-6)

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

  def test_uses_vehicle_model_limits_instead_of_tss2_rate_curve(self):
    ci = CarInterface(self.CP)
    state = update_state(ci, speed_ms=25.0, hud=CAMRY_HUD)
    self.assertAlmostEqual(state.vEgoRaw, 25.0, delta=0.05)

    with patch.object(ci.CC.tss3_request_transport, "control_generation_due", return_value=True):
      output, _ = ci.apply(control(20.0), 2_000_000_000)

    # the first request is limited from the measured angle, like panda
    step = output.steeringAngleDeg - (state.steeringAngleDeg + state.steeringAngleOffsetDeg)
    self.assertGreater(step, 0.20)
    self.assertLess(step, 0.22)

  def test_rejected_request_restarts_angle_limit_from_measured(self):
    ci = CarInterface(self.CP)
    state = update_state(ci, speed_ms=25.0, hud=CAMRY_HUD)
    measured = state.steeringAngleDeg + state.steeringAngleOffsetDeg
    transport = ci.CC.tss3_request_transport

    with patch.object(transport, "control_generation_due", return_value=True):
      for i in range(10):
        output, _ = ci.apply(control(20.0), 2_000_000_000 + i * 10_000_000)
      self.assertGreater(output.steeringAngleDeg, measured + 1.0)

      # panda rejected a request and restarted from the measured angle, the controller follows
      transport.request_rejected = True
      output, _ = ci.apply(control(20.0), 2_100_000_000)
    self.assertLess(output.steeringAngleDeg - measured, 0.22)

  def test_driver_nudge_inside_command_range_keeps_lateral(self):
    ci = CarInterface(self.CP)
    update_state(ci, speed_ms=20.0, hud=CAMRY_HUD)
    transport = ci.CC.tss3_request_transport
    with patch.object(transport, "control_generation_due", return_value=True):
      ci.CS.out.steeringTorque = 2.5
      ci.apply(control(0.0), 2_000_000_000)
    self.assertEqual(transport.pending_requests[-1].application[21] & 0x3F, LTA_LCA_REQUEST_ID)

  def test_driver_turn_past_command_range_stops_lateral(self):
    angle_max = CarControllerParams.TSS3_ANGLE_LIMITS.STEER_ANGLE_MAX
    for torque, angle, expected_angle in ((-2.0, -370.0, -angle_max), (1.8, 288.0, angle_max)):
      with self.subTest(angle=angle):
        ci = CarInterface(self.CP)
        update_state(ci, speed_ms=2.0, hud=CAMRY_HUD)
        transport = ci.CC.tss3_request_transport

        with patch.object(transport, "control_generation_due", return_value=True):
          ci.apply(control(0.0), 2_000_000_000)
          self.assertEqual(transport.pending_requests[-1].application[21] & 0x3F, LTA_LCA_REQUEST_ID)

          ci.CS.out.steeringTorque = torque
          ci.CS.out.steeringAngleDeg = angle - ci.CS.out.steeringAngleOffsetDeg
          output, _ = ci.apply(control(0.0), 2_010_000_000)
          application = transport.pending_requests[-1].application
          self.assertEqual(application[21] & 0x3F, 0)
          self.assertAlmostEqual(output.steeringAngleDeg, expected_angle, delta=1e-3)
          raw_angle = int.from_bytes(application[18:20], "big", signed=True)
          self.assertEqual(raw_angle, target_angle_deg_to_raw(expected_angle))

          # lateral resumes, rate limited from the last request, once the driver lets go inside the command range
          previous = output.steeringAngleDeg
          ci.CS.out.steeringTorque = 0.0
          ci.CS.out.steeringAngleDeg = float(np.clip(angle, -angle_max + 1, angle_max - 1)) - ci.CS.out.steeringAngleOffsetDeg
          output, _ = ci.apply(control(0.0), 2_020_000_000)
          self.assertEqual(transport.pending_requests[-1].application[21] & 0x3F, LTA_LCA_REQUEST_ID)
          self.assertLessEqual(abs(output.steeringAngleDeg - previous), CarControllerParams.TSS3_ANGLE_LIMITS.MAX_ANGLE_RATE + 1e-3)

  def test_host_request_plane_cancel_clones_native_brake_status_to_source_side(self):
    cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, relay_fingerprint(), [], True, False, False)
    ci = CarInterface(cp)
    update_state(ci, bus=0, source_bus=2, hud=CAMRY_HUD)
    _, sends = ci.apply(control(0.0, active=False, cancel=True), 2_000_000_000)
    self.assertIn((0x101, bytes.fromhex("8800000100000093"), 2), sends)

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


if __name__ == "__main__":
  unittest.main()
