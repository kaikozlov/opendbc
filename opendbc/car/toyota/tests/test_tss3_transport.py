import unittest

from opendbc.can import CANPacker
from opendbc.car.can_definitions import CanData
from opendbc.car.toyota.tss3 import (
  ADMIN_ADDR, ADMIN_BUS, DOWNSTREAM_BUS, NATIVE_08A_ADDR, ORACLE_BUS,
  ORACLE_REQUEST_ADDR, ORACLE_RESPONSE_ADDR, PANDA_REJECTED_OFFSET,
  PANDA_RETURNED_OFFSET, SOURCE_BUS, ToyotaTss3RequestTransport,
  build_host_application, build_oracle_transport,
)


KNOWN_APPLICATION = bytes.fromhex("0000000080000012ffae00ffae7fff007fff004b0000000000001e00")
KNOWN_TRAILER = bytes.fromhex("1d64e2a5")


def packets(nanos: int, *frames: CanData):
  return [(nanos, list(frames))]


def source_tick(nanos: int):
  return packets(nanos, CanData(NATIVE_08A_ADDR, bytes(32), SOURCE_BUS))


def response(nanos: int, sequence: int, status: int = 0, trailer: bytes = KNOWN_TRAILER):
  data = bytes((0xC9, sequence, status, sequence ^ 0xFF)) + trailer
  return packets(nanos, CanData(ORACLE_RESPONSE_ADDR, data, ORACLE_BUS))


class TestToyotaTss3RequestTransport(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("toyota_tss3_pt_generated")
    self.transport = ToyotaTss3RequestTransport(self.packer)

  def update_control(self, now_nanos: int, *, enabled: bool = True, lat_active: bool = True,
                     long_active: bool = True, angle: float = 0.0, accel: float = 0.0):
    return self.transport.update_control(
      enabled=enabled,
      lat_active=lat_active,
      target_angle_deg=angle,
      long_active=long_active,
      accel=accel,
      set_speed_kph=70.0,
      now_nanos=now_nanos,
    )

  def start_request(self, now_nanos: int = 1_000_000_000):
    self.transport.observe(source_tick(now_nanos), True)
    sends = self.update_control(now_nanos)
    self.assertFalse(any(msg.address == ADMIN_ADDR and msg.dat[0] == 7 for msg in sends))
    self.assertEqual(sum(msg.address == ORACLE_REQUEST_ADDR and msg.dat[0] == 0xC8 for msg in sends), 6)
    self.assertIsNotNone(self.transport.inflight)
    return self.transport.inflight.sequence

  def finish_handoff(self, now_nanos: int = 1_020_000_000):
    sequence = self.transport.inflight.sequence
    self.transport.observe(response(now_nanos, sequence), True)
    sends = self.update_control(now_nanos)
    self.assertEqual(sends[0], CanData(ADMIN_ADDR, bytes.fromhex("07c9a80100000000"), ADMIN_BUS))
    host = next(msg for msg in sends if msg.address == NATIVE_08A_ADDR)
    self.transport.observe(packets(now_nanos + 1, CanData(host.address, host.dat,
                                                         DOWNSTREAM_BUS + PANDA_RETURNED_OFFSET)), True)
    self.assertTrue(self.transport.active)
    return host

  def test_transport_is_six_application_only_fragments(self):
    frames = build_oracle_transport(1, KNOWN_APPLICATION)
    self.assertEqual(len(frames), 6)
    self.assertTrue(all(msg.address == ORACLE_REQUEST_ADDR and msg.src == ORACLE_BUS and len(msg.dat) == 8
                        for msg in frames))
    self.assertEqual([msg.dat[:2] for msg in frames], [bytes((0xC8, (i << 5) | 1)) for i in range(6)])
    self.assertEqual(b"".join(msg.dat[2:7] for msg in frames), KNOWN_APPLICATION + b"\0\0")
    with self.assertRaisesRegex(ValueError, "28 bytes"):
      build_oracle_transport(1, b"short")

  def test_host_application_uses_dbc_packer(self):
    application = build_host_application(
      self.packer,
      lat_active=True,
      target_angle_raw=-123,
      long_active=True,
      accel=-0.5,
      set_speed_kph=70.0,
      request_sequence=12,
    )
    self.assertEqual(application.hex(), "0000000880002d47fe0c46fe0c7fff007fffff85c00b100064000c00")

  def test_native_ticks_coalesce_while_one_signature_is_inflight(self):
    sequence = self.start_request()
    first_application = self.transport.inflight.application

    for nanos in (1_025_000_000, 1_050_000_000):
      self.transport.observe(source_tick(nanos), True)
      sends = self.update_control(nanos, angle=4.0, accel=-0.5)
      self.assertEqual(sends, [])
      self.assertEqual(self.transport.inflight.sequence, sequence)

    self.transport.observe(response(1_055_000_000, sequence), True)
    sends = self.update_control(1_055_000_000, angle=4.0, accel=-0.5)
    self.assertEqual(sum(msg.address == NATIVE_08A_ADDR for msg in sends), 1)
    self.assertEqual(sum(msg.address == ORACLE_REQUEST_ADDR and msg.dat[0] == 0xC8 for msg in sends), 6)
    self.assertNotEqual(self.transport.inflight.application, first_application)
    self.assertEqual(self.transport.inflight.application[8:10], (-500).to_bytes(2, "big", signed=True))

  def test_control_edge_discards_stale_signer_response(self):
    sequence = self.start_request()
    self.transport.observe(response(1_020_000_000, sequence), True)

    # The response was produced for active longitudinal control. A pedal or
    # disengagement edge updates longActive before CarController drains sends.
    sends = self.update_control(1_020_000_000, long_active=False)
    self.assertFalse(any(msg.address == NATIVE_08A_ADDR for msg in sends))

    self.transport.observe(source_tick(1_025_000_000), True)
    sends = self.update_control(1_025_000_000, long_active=False)
    self.assertEqual(sum(msg.address == ORACLE_REQUEST_ADDR and msg.dat[0] == 0xC8 for msg in sends), 6)
    self.assertEqual(self.transport.inflight.application[8:10], bytes(2))

  def test_timeout_retries_latest_control_without_historical_queue(self):
    first_sequence = self.start_request()
    sends = self.update_control(1_051_000_000, angle=3.0, accel=-0.75)
    self.assertEqual(sum(msg.address == ORACLE_REQUEST_ADDR and msg.dat[0] == 0xC8 for msg in sends), 6)
    self.assertNotEqual(self.transport.inflight.sequence, first_sequence)
    self.assertEqual(self.transport.inflight.application[8:10], (-750).to_bytes(2, "big", signed=True))

  def test_signer_error_retries_latest_control(self):
    first_sequence = self.start_request()
    self.transport.observe(response(1_020_000_000, first_sequence, status=2), True)
    sends = self.update_control(1_020_000_000, angle=2.0, accel=-0.25)
    self.assertEqual(sum(msg.dat[0] == 0xC8 for msg in sends), 6)
    self.assertNotEqual(self.transport.inflight.sequence, first_sequence)
    self.assertEqual(self.transport.inflight.application[8:10], (-250).to_bytes(2, "big", signed=True))

  def test_unmatched_signer_response_is_ignored(self):
    sequence = self.start_request()
    other_sequence = (sequence % 0x1F) + 1
    self.transport.observe(response(1_020_000_000, other_sequence), True)
    self.assertEqual(self.update_control(1_020_000_000), [])
    self.assertEqual(self.transport.inflight.sequence, sequence)

  def test_disable_releases_request_plane(self):
    self.start_request()
    self.finish_handoff()
    sends = self.update_control(1_030_000_000, enabled=False, lat_active=False, long_active=False)
    self.assertIn(CanData(ADMIN_ADDR, bytes.fromhex("07c9a80000000000"), ADMIN_BUS), sends)
    self.assertFalse(self.transport.active)

  def test_invalid_can_releases_request_plane(self):
    self.start_request()
    self.finish_handoff()
    self.transport.observe([], False)
    sends = self.update_control(1_030_000_000)
    self.assertIn(CanData(ADMIN_ADDR, bytes.fromhex("07c9a80000000000"), ADMIN_BUS), sends)
    self.assertEqual(self.transport.last_failure_reason, "can_invalid")
    self.assertFalse(self.transport.active)

  def test_handoff_reject_reports_unavailable_authority(self):
    self.start_request()
    sequence = self.transport.inflight.sequence
    self.transport.observe(response(1_020_000_000, sequence), True)
    sends = self.update_control(1_020_000_000)
    host = next(msg for msg in sends if msg.address == NATIVE_08A_ADDR)
    self.transport.observe(packets(1_021_000_000, CanData(host.address, host.dat,
                                                         DOWNSTREAM_BUS + PANDA_REJECTED_OFFSET)), True)
    self.assertEqual(self.transport.last_failure_reason, "handoff_host_frame_rejected")
    self.assertTrue(self.transport.authority_unavailable())


if __name__ == "__main__":
  unittest.main()
