import math
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, CanData, structs
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC, ToyotaFlags


# Representative incoming bus-1 frames copied verbatim from Span's tracked
# 2025 Corolla driving rlog (2026-07-29). The source log has MOCK carParams, so
# these prove the observed whole-vehicle wire format, not an exact F181 join.
SPAN_FRAMES = {
  0x00F: bytes.fromhex("162d0040d8a0a606"),
  0x025: bytes.fromhex("0ff800005fff0092000000000000000000000000000000000000000091f9fcc0"),
  0x0AA: bytes.fromhex("1abd1a6f1aba1a6f"),
  0x101: bytes.fromhex("8800003a000000cc"),
  0x116: bytes.fromhex("000200007a353eaa"),
  0x127: bytes.fromhex("001000000738d857"),
  0x176: bytes.fromhex("8800000000000007"),
  0x614: bytes.fromhex("000036300000ef04"),
  0x620: bytes.fromhex("0000000080000000"),
}


def fingerprint_on(bus: int) -> dict[int, dict[int, int]]:
  fp = {i: {} for i in range(8)}
  fp[bus] = {0x025: 32, 0x0AA: 8}
  return fp


class TestToyotaCorollaTSS3(unittest.TestCase):
  def test_platform_axes_and_passive_boundary(self):
    CP = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint_on(1), [], False, False, False)

    self.assertTrue(CP.flags & ToyotaFlags.TSS3)
    self.assertTrue(CP.flags & ToyotaFlags.SECOC)
    self.assertTrue(CP.flags & ToyotaFlags.TSS3_PT_BUS1)
    self.assertFalse(CP.flags & ToyotaFlags.TSS2)
    self.assertEqual(DBC[CAR.TOYOTA_COROLLA_TSS3][Bus.pt], "toyota_tss3_pt_generated")
    self.assertTrue(CP.dashcamOnly)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)
    self.assertEqual(CP.steerControlType, structs.CarParams.SteerControlType.angle)
    self.assertTrue(CP.radarUnavailable)
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertTrue(CP.secOcRequired)

  def test_relay_correct_topology_defaults_to_bus0(self):
    CP = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint_on(0), [], False, False, False)
    self.assertFalse(CP.flags & ToyotaFlags.TSS3_PT_BUS1)
    parsers = CarInterface.CarState.get_can_parsers(CP)
    self.assertEqual(parsers[Bus.pt].bus, 0)
    self.assertNotIn(Bus.cam, parsers)

  def test_unswapped_observed_topology_selects_bus1(self):
    CP = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint_on(1), [], False, False, False)
    parsers = CarInterface.CarState.get_can_parsers(CP)
    self.assertEqual(parsers[Bus.pt].bus, 1)
    self.assertNotIn(Bus.cam, parsers)

  def test_real_span_frames_decode_evidence_backed_carstate(self):
    CP = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint_on(1), [], False, False, False)
    CI = CarInterface(CP)

    packet = [CanData(address, dat, 1) for address, dat in SPAN_FRAMES.items()]
    CS = None
    # Repeat the representative wire values across a short interval so CANParser
    # alive checks converge while keeping every decoded payload source-real.
    for i in range(20):
      CS = CI.update([(1_000_000_000 + i * 10_000_000, packet)])
    assert CS is not None

    self.assertTrue(CS.canValid)
    self.assertTrue(CS.brakePressed)
    self.assertTrue(CS.gasPressed)
    self.assertEqual(CS.gearShifter, structs.CarState.GearShifter.drive)
    self.assertAlmostEqual(CS.steeringAngleDeg, -11.5)
    self.assertAlmostEqual(CS.steeringRateDeg, -1.0)
    self.assertGreater(CS.vEgoRaw, 0.0)
    self.assertFalse(CS.vehicleSensorsInvalid)
    self.assertFalse(CS.cruiseState.enabled)
    self.assertEqual(CS.steeringTorque, 0.0)
    self.assertEqual(CS.steeringTorqueEps, 0.0)
    self.assertFalse(CS.steerFaultTemporary)
    self.assertFalse(CS.steerFaultPermanent)

  def test_b6_receiver_fields_round_trip_without_enabling_sender(self):
    packer = CANPacker("toyota_tss3_pt_generated")
    parser = CANParser("toyota_tss3_pt_generated", [("TSS3_LATERAL_CONTROL", float('nan'))], 0)

    msg = packer.make_can_msg("TSS3_LATERAL_CONTROL", 0, {
      "TARGET_LATERAL_ID": 11,
      "TARGET_STEERING_ANGLE": 1024 / 17870,
      "SEQUENCE": 63,
    })
    parser.update([(1_000_000_000, [msg])])

    self.assertEqual(parser.vl["TSS3_LATERAL_CONTROL"]["TARGET_LATERAL_ID"], 11)
    self.assertEqual(parser.vl["TSS3_LATERAL_CONTROL"]["SEQUENCE"], 63)
    self.assertTrue(math.isclose(parser.vl["TSS3_LATERAL_CONTROL"]["TARGET_STEERING_ANGLE"], 1024 / 17870,
                                 rel_tol=0, abs_tol=1e-6))

  def test_controller_is_hard_noop(self):
    CP = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS3, fingerprint_on(1), [], False, False, False)
    CI = CarInterface(CP)
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.longActive = True
    CC.actuators.torque = 1.0
    CC.actuators.steeringAngleDeg = 300.0
    CC.actuators.accel = 2.0

    _, can_sends = CI.apply(CC.as_reader(), 1_000_000_000)
    self.assertEqual(can_sends, [])


if __name__ == "__main__":
  unittest.main()
