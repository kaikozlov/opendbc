"""Regression tests for the retained-evidence Camry port audit."""
import gzip
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from opendbc.car import CanData
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.tests.test_tss3_camry import CAMRY_COMMON, CAMRY_RADAR, fingerprint, update_state
from opendbc.car.toyota.toyotacan import toyota_e2e_p05_checksum
from opendbc.car.toyota.values import CAR
from opendbc.car.vehicle_model import VehicleModel


class TestCamryEvidenceAudit(unittest.TestCase):
  def setUp(self):
    self.cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], False, False, False)

  def test_learned_stiffness_is_relative_to_recorded_baseline(self):
    # Both working routes recorded this baseline. paramsd's STIFFNESS is a
    # multiplier of tireStiffnessFront/Rear, not CarParams.tireStiffnessFactor.
    self.assertAlmostEqual(self.cp.tireStiffnessFactor, 0.7933, places=4)
    vm = VehicleModel(self.cp)
    vm.update_params(1.0, 15.3)
    self.assertAlmostEqual(vm.cF, 163310.546875, delta=0.05)

  def test_radar_uses_recorded_bus_zero_on_stock_toyota_b(self):
    self.cp.radarUnavailable = False
    ri = RadarInterface(self.cp)
    self.assertEqual(ri.rcp.bus, 0)
    wrong_bus = ri.update([(1_000_000_000, [CanData(a, d, 1) for a, d in CAMRY_RADAR.items()])])
    self.assertTrue(wrong_bus is None or not wrong_bus.points)
    correct_bus = ri.update([(1_050_000_000, [CanData(a, d, 0) for a, d in CAMRY_RADAR.items()])])
    self.assertIsNotNone(correct_bus)
    self.assertGreater(len(correct_bus.points), 0)

  def test_radar_does_not_publish_geometry_without_matching_motion(self):
    self.cp.radarUnavailable = False
    ri = RadarInterface(self.cp)
    packets = [CanData(a, CAMRY_RADAR[a], ri.rcp.bus) for a in (0x180, 0x181, 0x182, 0x185)]
    result = ri.update([(1_000_000_000, packets)])
    self.assertTrue(result is None or not result.points)

  def test_radar_rejects_corrupt_geometry_instead_of_reusing_its_crc(self):
    self.cp.radarUnavailable = False
    ri = RadarInterface(self.cp)
    frames = dict(CAMRY_RADAR)
    corrupt = bytearray(frames[0x180])
    corrupt[4] ^= 1
    frames[0x180] = bytes(corrupt)
    result = ri.update([(1_000_000_000, [CanData(a, d, ri.rcp.bus) for a, d in frames.items()])])
    self.assertTrue(result is None or not result.points)

  def test_conventional_cruise_is_reported_via_the_normal_contract(self):
    for mode, non_adaptive in ((0x88, True), (0x90, True), (0x80, False), (0xA0, False), (0xC0, False), (0xE0, False)):
      with self.subTest(mode=hex(mode)):
        frames = dict(CAMRY_COMMON)
        frames[0x251] = bytes((mode,)) + frames[0x251][1:]
        ci = CarInterface(self.cp)
        with patch.dict(CAMRY_COMMON, frames):
          state = update_state(ci)
        self.assertEqual(state.cruiseState.nonAdaptive, non_adaptive)

  def test_planner_limits_match_the_camry_actuator_envelope(self):
    self.assertEqual(CarInterface.get_pid_accel_limits(self.cp, 15.0, 25.0), (-1.5, 1.3))

  def test_unqualified_radar_uses_the_normal_model_only_path(self):
    self.assertTrue(self.cp.radarUnavailable)
    self.assertIsNone(RadarInterface(self.cp).rcp)

  @staticmethod
  def radar_cycle(counter, *, empty=False):
    frames = {}
    for address, original in CAMRY_RADAR.items():
      data = bytearray(original)
      data[3] = (data[3] + counter - data[2]) & 0xFF
      data[2] = counter
      if empty:
        data[4:60] = (bytes.fromhex("fff8000000ffff") * 8) if address < 0x183 else bytes(56)
      data[:2] = toyota_e2e_p05_checksum(address, data).to_bytes(2, "little")
      frames[address] = bytes(data)
    return frames

  def test_object_cycle_wrap_preserves_live_track_identity(self):
    self.cp.radarUnavailable = False
    ri = RadarInterface(self.cp)
    track_ids = None
    for i, counter in enumerate((254, 255, 0, 1)):
      frames = self.radar_cycle(counter)
      result = ri.update([(1_000_000_000 + i * 50_000_000, [CanData(a, d, 0) for a, d in frames.items()])])
      self.assertIsNotNone(result)
      self.assertFalse(result.errors.canError)
      ids = {p.trackId for p in result.points}
      self.assertEqual(len(ids), 8)
      if track_ids is not None:
        self.assertEqual(ids, track_ids)
      track_ids = ids

  def test_empty_slot_retirement_does_not_reuse_track_id(self):
    self.cp.radarUnavailable = False
    ri = RadarInterface(self.cp)
    outputs = []
    for i, empty in enumerate((False, True, False)):
      result = ri.update([(1_000_000_000 + i * 50_000_000,
                          [CanData(a, d, 0) for a, d in self.radar_cycle(i, empty=empty).items()])])
      outputs.append({p.trackId for p in result.points})
    self.assertEqual(outputs[1], set())
    self.assertEqual(len(outputs[2]), 8)
    self.assertTrue(outputs[0].isdisjoint(outputs[2]))

  def test_mismatched_cycles_and_truncated_packets_do_not_make_points(self):
    self.cp.radarUnavailable = False
    for truncated in (False, True):
      ri = RadarInterface(self.cp)
      frames = self.radar_cycle(20)
      frames[0x183] = frames[0x183][:8] if truncated else self.radar_cycle(21)[0x183]
      result = ri.update([(1_000_000_000, [CanData(a, d, 0) for a, d in frames.items()])])
      self.assertTrue(result is None or not result.points)

  def test_recorded_high_closing_speed_does_not_wrap_into_positive_speed(self):
    self.cp.radarUnavailable = False
    # Recorded motion bytes from route 3b, segment 54. This is a decoding
    # vector, deliberately placed in the first fixture slot, not a new drive.
    for flags in (0x00, 0x40, 0x80, 0xC0):
      ri = RadarInterface(self.cp)
      frames = self.radar_cycle(16)
      data = bytearray(frames[0x183])
      data[4:11] = bytes.fromhex("0836e400809a02")
      data[5] = (data[5] & 0x3F) | flags
      data[:2] = toyota_e2e_p05_checksum(0x183, data).to_bytes(2, "little")
      frames[0x183] = bytes(data)
      result = ri.update([(1_000_000_000, [CanData(a, d, 0) for a, d in frames.items()])])
      self.assertAlmostEqual(result.points[0].dRel, 9.33, places=2)
      self.assertAlmostEqual(result.points[0].yRel, .64, places=2)
      self.assertAlmostEqual(result.points[0].vRel, -58.3, places=2)

  def test_original_native_cadence_and_dead_eps_liveness(self):
    root = Path(__file__).parent / "fixtures/camry_20260915_audit"
    manifest = json.loads((root / "manifest.json").read_text())
    for source in manifest["sources"]:
      with self.subTest(source=source["label"]):
        path = root / source["fixture"]
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), source["fixture_sha256"])
        translation = {int(address, 16): buses for address, buses in source["translation"].items()}
        ci = CarInterface(self.cp)
        observed = 0
        for line in gzip.decompress(path.read_bytes()).decode().splitlines():
          nanos, frames = json.loads(line)
          packets = [CanData(address, bytes.fromhex(data), translation[address][1] if translation else bus)
                     for address, data, bus in frames]
          state = ci.update([(nanos + 1, packets)])
          if nanos < manifest["warmup_seconds"] * 1_000_000_000:
            continue
          observed += 1
          self.assertEqual(state.canValid, source["label"] == "working_eps_repin")
          self.assertTrue(state.cruiseState.nonAdaptive)
          self.assertFalse(state.vehicleSensorsInvalid)
          if source["label"] == "stock_harness_eps_absent":
            missing = {msg.address for parser in ci.can_parsers.values() for msg in parser.message_states.values()
                       if not msg.ignore_alive and not msg.timestamps}
            self.assertEqual(missing, {0x030})
        self.assertGreater(observed, 450)


if __name__ == '__main__':
  unittest.main()
