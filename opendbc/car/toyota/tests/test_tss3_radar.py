import binascii
import unittest

from opendbc.car import CanData
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.values import CAR, ToyotaSafetyFlags


def checksum(address, data):
  data[:2] = binascii.crc_hqx(data[2:] + address.to_bytes(2, "little"), 0xFFFF).to_bytes(2, "little")
  return bytes(data)


def cycle(counter, state=2, new=False, ended=False, occupied=True, cycle_byte=None, bank=0, slot=0):
  """Independent byte-level source fixture, without using the production packer."""
  frames = {}
  for address in range(0x180, 0x186):
    data = bytearray(64)
    data[2] = counter & 0xFF
    data[3] = (counter + 6 if cycle_byte is None else cycle_byte) & 0xFF
    if address < 0x183:
      data[4:60] = bytes.fromhex("fff8000000ffff") * 8
    frames[address] = data
  offset = 4 + slot * 7
  if occupied:
    frames[0x180 + bank][offset:offset + 7] = bytes.fromhex("0fa0019000ffff")  # 20 m, +1 m
  frames[0x183 + bank][offset + 1:offset + 3] = bytes.fromhex("3fd8")  # -1 m/s
  frames[0x183 + bank][offset + 4] = 0x80 if new else 0
  frames[0x183 + bank][offset + 5] = (0x10 if ended else 0) | state
  return [CanData(a, checksum(a, d), 1) for a, d in frames.items()]


class TestToyotaTSS3Radar(unittest.TestCase):
  def setUp(self):
    cp = CarInterface.get_non_essential_params(CAR.TOYOTA_CAMRY_TSS3)
    cp.radarUnavailable = False
    self.ri = RadarInterface(cp)
    self.time = 1_000_000_000

  def update(self, frames):
    result = self.ri.update([(self.time, frames)])
    self.time += 50_000_000
    return result

  def test_geometry_and_all_slots(self):
    for bank in range(3):
      for slot in range(8):
        rr = self.update(cycle(bank * 8 + slot, new=True, bank=bank, slot=slot))
        self.assertFalse(rr.errors.canError)
        self.assertEqual(len(rr.points), 1)
        point = rr.points[0]
        self.assertAlmostEqual(point.dRel, 20)
        self.assertAlmostEqual(point.yRel, 1)
        self.assertAlmostEqual(point.vRel, -1)

  def test_same_object_keeps_track_id(self):
    first = self.update(cycle(0, new=True)).points[0].trackId
    for counter in range(1, 10):
      self.assertEqual(self.update(cycle(counter)).points[0].trackId, first)

  def test_new_flag_replaces_occupied_slot_without_empty_frame(self):
    first = self.update(cycle(0)).points[0].trackId
    second = self.update(cycle(1, new=True)).points[0].trackId
    self.assertNotEqual(first, second)
    self.assertEqual(self.update(cycle(2)).points[0].trackId, second)

  def test_ended_and_new_flags_replace_instead_of_discarding_new_object(self):
    first = self.update(cycle(0)).points[0].trackId
    rr = self.update(cycle(1, new=True, ended=True))
    self.assertEqual(len(rr.points), 1)
    self.assertNotEqual(rr.points[0].trackId, first)

  def test_deletion_and_unqualified_retained_geometry_retire_track(self):
    for state, occupied, ended in ((0, False, True), (0, True, False), (2, False, False), (2, True, True)):
      with self.subTest(state=state, occupied=occupied, ended=ended):
        self.setUp()
        first = self.update(cycle(0)).points[0].trackId
        self.assertEqual(list(self.update(cycle(1, state=state, occupied=occupied, ended=ended)).points), [])
        self.assertNotEqual(self.update(cycle(2)).points[0].trackId, first)

  def test_batched_cycles_preserve_intermediate_lifecycle_events(self):
    first = self.update(cycle(0)).points[0].trackId
    rr = self.ri.update([(self.time, cycle(1, new=True, ended=True)),
                         (self.time + 50_000_000, cycle(2))])
    self.assertEqual(len(rr.points), 1)
    self.assertNotEqual(rr.points[0].trackId, first)
    self.assertEqual(rr.points[0].trackId, first + 1)

  def test_lifecycle_in_separate_publications(self):
    self.update(cycle(0))
    first = self.ri.pts[0].trackId
    frames = cycle(1, new=True, ended=True)
    self.assertIsNone(self.ri.update([(self.time, frames[:3])]))
    rr = self.ri.update([(self.time + 1_000_000, frames[3:])])
    self.assertNotEqual(rr.points[0].trackId, first)

  def test_first_source_cycle_can_span_publications(self):
    frames = cycle(0)
    self.assertIsNone(self.ri.update([(self.time, frames[:3])]))
    rr = self.ri.update([(self.time + 1_000_000, frames[3:])])
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 1)

  def test_arbitrary_order_within_complete_source_cycle(self):
    frames = cycle(0)
    rr = self.update(list(reversed(frames)))
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 1)

  def test_cycle_wrap_preserves_identity_but_gap_does_not(self):
    first = self.update(cycle(254)).points[0].trackId
    self.assertEqual(self.update(cycle(255)).points[0].trackId, first)
    self.assertEqual(self.update(cycle(0)).points[0].trackId, first)
    self.assertNotEqual(self.update(cycle(2)).points[0].trackId, first)

  def test_duplicate_complete_cycle_does_not_repeat_new_track(self):
    rr = self.update(cycle(0, new=True))
    first = rr.points[0].trackId
    self.assertIsNone(self.update(cycle(0, new=True)))
    self.assertEqual(self.update(cycle(1)).points[0].trackId, first)

  def test_mismatched_cycle_is_never_fused(self):
    self.update(cycle(0))
    frames = cycle(1)
    data = bytearray(frames[0].dat)
    data[3] += 1
    frames[0] = CanData(0x180, checksum(0x180, data), 0)
    self.assertIsNone(self.update(frames))

  def test_corrupt_and_truncated_frames_do_not_fuse_with_later_motion(self):
    for truncated in (True, False):
      with self.subTest(truncated=truncated):
        self.setUp()
        self.update(cycle(0))
        frames = cycle(1, new=True)
        data = bytearray(frames[0].dat)
        if truncated:
          data = data[:8]
        else:
          data[0] ^= 1
        frames[0] = CanData(0x180, bytes(data), 0)
        self.assertIsNone(self.update(frames))
        rr = self.update(cycle(2))
        self.assertEqual(len(rr.points), 1)
        self.assertNotEqual(rr.points[0].trackId, 0)

  def test_total_radar_loss_reports_error_and_clears_stale_points(self):
    first = self.update(cycle(0)).points[0].trackId
    results = []
    for _ in range(25):
      rr = self.update([])
      if rr is not None:
        results.append(rr)
    self.assertTrue(results)
    self.assertTrue(results[-1].errors.canError)
    self.assertEqual(list(results[-1].points), [])
    self.assertFalse(self.ri.pts)
    rr = self.update(cycle(1))
    self.assertNotEqual(rr.points[0].trackId, first)

  def test_relay_correct_camry_radar_uses_unsplit_bus1(self):
    cp = CarInterface.get_non_essential_params(CAR.TOYOTA_CAMRY_TSS3)
    cp.radarUnavailable = False
    cp.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.TSS3_08A_HOST.value
    ri = RadarInterface(cp)
    self.assertEqual(ri.rcp.bus, 1)
    frames = [CanData(frame.address, frame.dat, 1) for frame in cycle(0)]
    rr = ri.update([(self.time, frames)])
    self.assertIsNotNone(rr)
    self.assertFalse(rr.errors.canError)
    self.assertEqual(len(rr.points), 1)

  def test_unrelated_and_wrong_bus_traffic_cannot_publish_radar(self):
    frames = [CanData(frame.address, frame.dat, 0) for frame in cycle(0)]
    rr = self.update(frames)
    self.assertIsNone(rr)
    self.assertFalse(self.ri.pts)


if __name__ == '__main__':
  unittest.main()
