"""Retained Corolla wire-format evidence replayed on the canonical TSS3 repin."""
import unittest

from opendbc.can import CANDefine, CANParser
from opendbc.car import CanData
from opendbc.car.toyota.tss3 import TSS3_CHASSIS_BUS


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

ALBINO_GEAR = {
  "P": bytes.fromhex("8000010074d0de47"),
  "R": bytes.fromhex("400001006f306582"),
  "D": bytes.fromhex("100001005fc18f5f"),
}


class TestToyotaTss3SharedDbc(unittest.TestCase):
  def test_retained_corolla_state_decodes_with_shared_dbc(self):
    # The raw Span capture used an unrepinned stock-harness bus. Bus identity is
    # not part of this DBC regression: all maintained TSS3 integration uses the
    # canonical repinned chassis bus.
    parser = CANParser("toyota_tss3_pt_generated", [(address, 0) for address in (0x025, 0x030, 0x101, 0x116, 0x127)], TSS3_CHASSIS_BUS)
    parser.update([(1_000_000_000, [CanData(a, d, TSS3_CHASSIS_BUS) for a, d in SPAN_FRAMES.items()])])
    angle = parser.vl["STEER_ANGLE_SENSOR"]
    self.assertAlmostEqual(angle["STEER_ANGLE"] + angle["STEER_FRACTION"], -11.5)
    self.assertAlmostEqual(angle["STEER_RATE"], -1.0)
    torque = parser.vl["TSS3_EPS_TELEMETRY"]
    self.assertAlmostEqual(torque["STEERING_WHEEL_TORQUE_COARSE"] + torque["STEERING_WHEEL_TORQUE_FINE"], 1.06)
    self.assertTrue(parser.vl["BRAKE_MODULE"]["BRAKE_PRESSED"])
    self.assertGreater(parser.vl["GAS_PEDAL"]["GAS_PEDAL_USER"], 0)

  def test_retained_corolla_nonhybrid_gear_evidence(self):
    parser = CANParser("toyota_tss3_pt_generated", [("TSS3_GEAR_PACKET", 0)], TSS3_CHASSIS_BUS)
    names = CANDefine("toyota_tss3_pt_generated").dv["TSS3_GEAR_PACKET"]["GEAR"]
    for i, (expected, data) in enumerate(ALBINO_GEAR.items()):
      parser.update([(1_000_000_000 + i * 10_000_000, [CanData(0x3BF, data, TSS3_CHASSIS_BUS)])])
      self.assertEqual(names[int(parser.vl["TSS3_GEAR_PACKET"]["GEAR"])], expected)
