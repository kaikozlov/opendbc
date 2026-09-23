import unittest

from opendbc.can import CANPacker
from opendbc.car.can_definitions import CanData
from opendbc.car.toyota.tss3 import (
  CONTROL_REQUEST_ADDR, GENERATION_INTERVAL_NS, PANDA_REJECTED_OFFSET, PANDA_RETURNED_OFFSET, SIGNER_ADDR,
  SIGNER_MAX_PENDING, SIGNER_RESPONSE_ADDR, SIGNER_SEQUENCE_MAX, TSS3_AUX_BUS, TSS3_CHASSIS_BUS, TSS3_SOURCE_BUS,
  ToyotaTss3RequestTransport, build_host_application, build_signer_requests, make_admin_msg,
)


KNOWN_APPLICATION = bytes.fromhex("0000000080000012ffae00ffae7fff007fff004b0000000000001e00")
KNOWN_TRAILER = bytes.fromhex("1d64e2a5")


def packets(nanos: int, *frames: CanData):
  return [(nanos, list(frames))]


def source_tick(nanos: int):
  return packets(nanos, CanData(CONTROL_REQUEST_ADDR, bytes(32), TSS3_SOURCE_BUS))


def response(nanos: int, sequence: int, status: int = 0, trailer: bytes = KNOWN_TRAILER):
  data = bytes((0xC9, sequence, status, sequence ^ 0xFF)) + trailer
  return packets(nanos, CanData(SIGNER_RESPONSE_ADDR, data, TSS3_CHASSIS_BUS))


def signer_request_frames(sends: list[CanData]) -> list[CanData]:
  return [msg for msg in sends if msg.address == SIGNER_ADDR and 8 <= (msg.dat[0] >> 4) <= 0xB]


def request_sequence_from_sends(sends: list[CanData]) -> int:
  frames = signer_request_frames(sends)
  low = next(msg.dat[0] & 0x0F for msg in frames if msg.dat[0] >> 4 == 8)
  high = next(msg.dat[0] & 0x0F for msg in frames if msg.dat[0] >> 4 == 9)
  return (high << 4) | low


class TestToyotaTss3RequestTransport(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("toyota_tss3_pt_generated")
    self.transport = ToyotaTss3RequestTransport(self.packer)
    self.transport.observe([], True)

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
    sends = self.update_control(now_nanos)
    self.assertFalse(any(msg.address == SIGNER_ADDR and msg.dat[0] == 7 for msg in sends))
    self.assertEqual(len(signer_request_frames(sends)), 4)
    self.assertEqual(len(self.transport.pending_requests), 1)
    return request_sequence_from_sends(sends)

  def finish_handoff(self, now_nanos: int = 1_000_000_000):
    sequence = self.start_request(now_nanos)
    self.transport.observe(response(now_nanos + 15_000_000, sequence), True)
    sends = self.update_control(now_nanos + 20_000_000)
    self.assertEqual(sends[0], CanData(SIGNER_ADDR, bytes.fromhex("07c9a80100000000"), TSS3_AUX_BUS))
    host = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.transport.observe(packets(now_nanos + 20_000_001, CanData(host.address, host.dat,
                                                                   TSS3_CHASSIS_BUS + PANDA_RETURNED_OFFSET)), True)
    self.assertTrue(self.transport.active)
    return host

  def test_transport_is_four_application_only_fragments(self):
    frames = build_signer_requests(0xA7, KNOWN_APPLICATION)
    self.assertEqual(len(frames), 4)
    self.assertTrue(all(msg.address == SIGNER_ADDR and msg.src == TSS3_CHASSIS_BUS and len(msg.dat) == 8
                        for msg in frames))
    self.assertEqual([msg.dat[0] for msg in frames], [0x87, 0x9A, 0xA7, 0xBA])
    self.assertEqual(b"".join(msg.dat[1:] for msg in frames), KNOWN_APPLICATION)

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

  def test_stock_longitudinal_application_only_replaces_lateral_fields(self):
    stock = bytes.fromhex("00000008a0043543ff380cff067ffe007ffdff0123d255342132a57e")
    application = build_host_application(
      self.packer,
      lat_active=True,
      target_angle_raw=-123,
      long_active=False,
      accel=2.0,
      set_speed_kph=70.0,
      request_sequence=12,
      stock_application=stock,
    )

    for index in (*range(18), 20, 22, 23, 27):
      self.assertEqual(application[index], stock[index], index)
    self.assertEqual(application[18:20], (-123).to_bytes(2, "big", signed=True))
    self.assertEqual(application[21] & 0x3F, 11)
    self.assertEqual(application[21] & 0xC0, stock[21] & 0xC0)
    self.assertEqual(application[24:26], bytes((100, 0)))
    self.assertEqual(application[26] & 0x3F, 12)
    self.assertEqual(application[26] & 0xC0, stock[26] & 0xC0)

  def test_stock_longitudinal_transport_waits_for_frc_application(self):
    transport = ToyotaTss3RequestTransport(self.packer, stock_longitudinal=True)
    transport.observe([], True)
    self.assertFalse(transport.control_generation_due(
      enabled=True, lat_active=True, long_active=False, now_nanos=1_000_000_000,
    ))
    self.assertEqual(transport.update_control(
      enabled=True, lat_active=True, target_angle_deg=0.0, long_active=False,
      accel=0.0, set_speed_kph=70.0, now_nanos=1_000_000_000,
    ), [])

    stock = bytes.fromhex("00000008a0043543ff380cff067ffe007ffdff0123d255342132a57e")
    transport.observe(packets(1_005_000_000, CanData(CONTROL_REQUEST_ADDR, stock + bytes(4), TSS3_SOURCE_BUS)), True)
    self.assertTrue(transport.control_generation_due(
      enabled=True, lat_active=True, long_active=False, now_nanos=1_010_000_000,
    ))
    sends = transport.update_control(
      enabled=True, lat_active=True, target_angle_deg=0.0, long_active=False,
      accel=0.0, set_speed_kph=70.0, now_nanos=1_010_000_000,
    )
    self.assertEqual(len(signer_request_frames(sends)), 4)
    self.assertEqual(transport.pending_requests[0].application[6:18], stock[6:18])

  def test_stock_longitudinal_40hz_source_does_not_gate_100hz_host_publication(self):
    transport = ToyotaTss3RequestTransport(self.packer, stock_longitudinal=True)
    stock = bytes.fromhex("00000008a0043543ff380cff067ffe007ffdff0123d255342132a57e")
    start_ns = 1_000_000_000
    transport.observe(packets(start_ns, CanData(CONTROL_REQUEST_ADDR, stock + bytes(4), TSS3_SOURCE_BUS)), True)

    sends = transport.update_control(
      enabled=True, lat_active=True, target_angle_deg=0.0, long_active=False,
      accel=0.0, set_speed_kph=70.0, now_nanos=start_ns,
    )
    seq0 = request_sequence_from_sends(sends)
    transport.observe(response(start_ns + 5_000_000, seq0), True)

    sends = transport.update_control(
      enabled=True, lat_active=True, target_angle_deg=0.1, long_active=False,
      accel=0.0, set_speed_kph=70.0, now_nanos=start_ns + 10_000_000,
    )
    host0 = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    seq1 = request_sequence_from_sends(sends)
    transport.observe(packets(start_ns + 10_000_001, CanData(
      host0.address, host0.dat, TSS3_CHASSIS_BUS + PANDA_RETURNED_OFFSET,
    )), True)
    transport.observe(response(start_ns + 15_000_000, seq1), True)

    # No second FRC frame has arrived, but the next signed host generation is
    # still published 10 ms later with a fresh application sequence.
    sends = transport.update_control(
      enabled=True, lat_active=True, target_angle_deg=0.2, long_active=False,
      accel=0.0, set_speed_kph=70.0, now_nanos=start_ns + 20_000_000,
    )
    host1 = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual((host0.dat[26] & 0x3F, host1.dat[26] & 0x3F), (0, 1))
    self.assertEqual(host0.dat[6:18], stock[6:18])
    self.assertEqual(host1.dat[6:18], stock[6:18])

  def test_controller_clock_generates_without_a_native_08a_tick(self):
    self.assertTrue(self.transport.control_generation_due(
      enabled=True, lat_active=True, long_active=True, now_nanos=1_000_000_000,
    ))
    sends = self.update_control(1_000_000_000)
    self.assertEqual(len(signer_request_frames(sends)), 4)
    first_sequence = request_sequence_from_sends(sends)

    # Native source traffic is still parser/liveness input, but it neither
    # creates nor gates host generations.
    self.transport.observe(source_tick(1_005_000_000), True)
    self.assertEqual(len(self.transport.pending_requests), 1)

    self.assertTrue(self.transport.control_generation_due(
      enabled=True, lat_active=True, long_active=True, now_nanos=1_010_000_000,
    ))
    sends = self.update_control(1_010_000_000, angle=1.0, accel=-0.25)
    self.assertEqual(len(signer_request_frames(sends)), 4)
    self.assertNotEqual(request_sequence_from_sends(sends), first_sequence)
    self.assertEqual(len(self.transport.pending_requests), 2)

  def test_100hz_pipeline_allows_multiple_signatures_in_flight(self):
    sequences = []
    for i in range(4):
      now = 1_000_000_000 + i * 10_000_000
      sends = self.update_control(now, angle=float(i), accel=-0.1 * i)
      self.assertEqual(len(signer_request_frames(sends)), 4)
      sequences.append(request_sequence_from_sends(sends))

    self.assertEqual(len(self.transport.pending_requests), 4)
    self.assertEqual([request.sequence for request in self.transport.pending_requests], sequences)
    self.assertEqual([request.application[26] for request in self.transport.pending_requests], [0, 1, 2, 3])

  def test_pipeline_publishes_at_100hz_after_signer_latency_is_hidden(self):
    seq0 = self.start_request(1_000_000_000)

    sends = self.update_control(1_010_000_000, angle=1.0)
    seq1 = request_sequence_from_sends(sends)

    self.transport.observe(response(1_019_000_000, seq0), True)
    sends = self.update_control(1_020_000_000, angle=2.0)
    host0 = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual(host0.dat[26], 0)
    seq2 = request_sequence_from_sends(sends)

    # Confirm the first host publication so subsequent signed frames can flow.
    self.transport.observe(packets(1_020_000_001, CanData(host0.address, host0.dat,
                                                         TSS3_CHASSIS_BUS + PANDA_RETURNED_OFFSET)), True)

    self.transport.observe(response(1_029_000_000, seq1), True)
    sends = self.update_control(1_030_000_000, angle=3.0)
    host1 = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual(host1.dat[26], 1)
    seq3 = request_sequence_from_sends(sends)

    self.transport.observe(response(1_039_000_000, seq2), True)
    sends = self.update_control(1_040_000_000, angle=4.0)
    host2 = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual(host2.dat[26], 2)

    self.assertNotEqual(seq0, seq1)
    self.assertNotEqual(seq1, seq2)
    self.assertNotEqual(seq2, seq3)

  def test_later_response_supersedes_one_missing_response_without_stalling(self):
    seq0 = self.start_request(1_000_000_000)
    sends = self.update_control(1_010_000_000, angle=1.0, accel=-0.25)
    seq1 = request_sequence_from_sends(sends)

    # Response 0 is lost. Response 1 proves the resident advanced past it, so
    # publish generation 1 on the next ordinary controller tick rather than
    # waiting 50 ms and retrying stale control.
    self.transport.observe(response(1_019_000_000, seq1), True)
    sends = self.update_control(1_020_000_000, angle=2.0, accel=-0.5)
    host = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual(host.dat[26], 1)
    self.assertEqual(host.dat[8:10], (-250).to_bytes(2, "big", signed=True))
    self.assertNotIn(seq0, self.transport.requests_by_sequence)

  def test_missing_response_preserves_generation_timing_after_handoff(self):
    self.finish_handoff(1_000_000_000)
    first_host_send_ns = 1_020_000_000
    missing = self.transport.pending_requests[0].sequence

    sends = self.update_control(1_030_000_000, angle=1.0)
    later = request_sequence_from_sends(sends)
    self.assertNotEqual(missing, later)

    # The later response proves the missing generation was consumed. It is
    # ready only 19 ms after generation 0 was published, so generation 2 must
    # wait for its full two-generation (20 ms) command interval.
    self.transport.observe(response(1_038_000_000, later), True)
    sends = self.update_control(first_host_send_ns + (2 * GENERATION_INTERVAL_NS) - 1, angle=2.0)
    self.assertFalse(any(msg.address == CONTROL_REQUEST_ADDR for msg in sends))

    sends = self.update_control(first_host_send_ns + (2 * GENERATION_INTERVAL_NS), angle=2.0)
    host = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual(host.dat[26], 2)

  def test_generation_waits_for_real_pipeline_capacity(self):
    # Fill the request pipeline, then make its head signed but not yet eligible
    # for publication. A skipped request sequence needs 20 ms since the last
    # host frame; merely having its trailer must not advance CarController's
    # angle limiter when the request cannot be removed and replaced this tick.
    for i in range(SIGNER_MAX_PENDING):
      self.update_control(1_000_000_000 + i * GENERATION_INTERVAL_NS, angle=float(i))

    # A state edge or an invalid request behind the head does not itself make
    # room in the bounded deque. Both used to make the admission predicate say
    # yes even though _queue_signing_latest would refuse the new generation.
    self.transport.pending_requests[-1].failed = True
    self.assertFalse(self.transport.control_generation_due(
      enabled=True, lat_active=True, long_active=False, now_nanos=1_080_000_000,
    ))
    self.transport.pending_requests[-1].failed = False

    head = self.transport.pending_requests[0]
    head.trailer = KNOWN_TRAILER
    self.transport.last_host_request_sequence = 62
    self.transport.last_host_send_ns = 1_100_000_000

    self.assertFalse(self.transport.control_generation_due(
      enabled=True, lat_active=True, long_active=True,
      now_nanos=1_100_000_000 + 2 * GENERATION_INTERVAL_NS - 1,
    ))
    self.assertTrue(self.transport.control_generation_due(
      enabled=True, lat_active=True, long_active=True,
      now_nanos=1_100_000_000 + 2 * GENERATION_INTERVAL_NS,
    ))

    # Even an otherwise publishable head cannot free capacity while the first
    # handoff is waiting for Panda's transmit confirmation.
    self.transport.arm_pending = True
    self.assertFalse(self.transport.control_generation_due(
      enabled=True, lat_active=True, long_active=True,
      now_nanos=1_100_000_000 + 2 * GENERATION_INTERVAL_NS,
    ))

  def test_ready_responses_are_preserved_in_order(self):
    seq0 = self.start_request(1_000_000_000)
    seq1 = request_sequence_from_sends(self.update_control(1_010_000_000, angle=1.0))

    frames = [
      CanData(SIGNER_RESPONSE_ADDR, bytes((0xC9, seq0, 0, seq0 ^ 0xFF)) + KNOWN_TRAILER, TSS3_CHASSIS_BUS),
      CanData(SIGNER_RESPONSE_ADDR, bytes((0xC9, seq1, 0, seq1 ^ 0xFF)) + bytes.fromhex("2d64e2a5"), TSS3_CHASSIS_BUS),
    ]
    self.transport.observe(packets(1_019_000_000, *frames), True)

    sends = self.update_control(1_020_000_000)
    host0 = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual(host0.dat[26], 0)
    self.transport.observe(packets(1_020_000_001, CanData(host0.address, host0.dat,
                                                         TSS3_CHASSIS_BUS + PANDA_RETURNED_OFFSET)), True)

    sends = self.update_control(1_030_000_000)
    host1 = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual(host1.dat[26], 1)

  def test_signer_error_skips_failed_generation_instead_of_retrying_it(self):
    seq0 = self.start_request(1_000_000_000)
    seq1 = request_sequence_from_sends(self.update_control(1_010_000_000, angle=1.0, accel=-0.25))

    self.transport.observe(response(1_018_000_000, seq0, status=2), True)
    self.transport.observe(response(1_019_000_000, seq1), True)
    sends = self.update_control(1_020_000_000, angle=2.0, accel=-0.5)

    host = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual(host.dat[26], 1)
    self.assertEqual(host.dat[8:10], (-250).to_bytes(2, "big", signed=True))
    self.assertNotIn(seq0, self.transport.requests_by_sequence)

  def test_axis_activity_edge_preserves_combined_request_pipeline(self):
    sends = self.update_control(1_000_000_000, accel=-0.5)
    seq0 = request_sequence_from_sends(sends)

    # 0x08A carries both axes. A gas-override longActive edge must enqueue the
    # new zero-accel state without invalidating lateral or interrupting the
    # already ordered signing pipeline.
    sends = self.update_control(1_010_000_000, long_active=False)
    seq1 = request_sequence_from_sends(sends)
    self.assertNotEqual(seq0, seq1)
    self.assertEqual(len(self.transport.pending_requests), 2)
    self.assertEqual(self.transport.pending_requests[0].application[8:10], (-500).to_bytes(2, "big", signed=True))
    self.assertEqual(self.transport.pending_requests[1].application[8:10], bytes(2))

    self.transport.observe(response(1_015_000_000, seq0), True)
    self.transport.observe(response(1_019_000_000, seq1), True)
    sends = self.update_control(1_020_000_000, long_active=False)
    host0 = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual(host0.dat[8:10], (-500).to_bytes(2, "big", signed=True))
    self.transport.observe(packets(1_020_000_001, CanData(host0.address, host0.dat,
                                                         TSS3_CHASSIS_BUS + PANDA_RETURNED_OFFSET)), True)

    sends = self.update_control(1_030_000_000, long_active=False)
    host1 = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.assertEqual(host1.dat[8:10], bytes(2))

  def test_all_axis_activity_edges_keep_the_same_control_epoch(self):
    self.update_control(1_000_000_000, lat_active=True, long_active=True)
    epoch = self.transport.control_epoch
    first_request = self.transport.pending_requests[0]

    for i, (lat_active, long_active) in enumerate(((False, True), (False, False), (True, False), (True, True)), start=1):
      sends = self.update_control(1_000_000_000 + i * 10_000_000,
                                  lat_active=lat_active, long_active=long_active)
      self.assertEqual(self.transport.control_epoch, epoch)
      self.assertIs(self.transport.pending_requests[0], first_request)
      self.assertNotIn(make_admin_msg(False), sends)

  def test_unmatched_signer_response_is_ignored(self):
    seq = self.start_request(1_000_000_000)
    other = (seq % SIGNER_SEQUENCE_MAX) + 1
    self.transport.observe(response(1_005_000_000, other), True)
    self.assertIn(seq, self.transport.requests_by_sequence)
    self.assertIsNone(self.transport.requests_by_sequence[seq].trailer)

  def test_watchdog_starts_when_signing_can_actually_start(self):
    self.transport.observe([], False)
    sends = self.update_control(1_000_000_000)
    self.assertEqual(sends, [])
    self.assertEqual(self.transport.control_started_ns, 0)

    # Recovering CAN well after 90 ms must start acquisition normally rather
    # than inheriting a watchdog deadline from the earlier enable edge.
    self.transport.observe([], True)
    sends = self.update_control(2_000_000_000)
    self.assertEqual(len(signer_request_frames(sends)), 4)
    self.assertEqual(self.transport.control_started_ns, 2_000_000_000)

  def test_pipeline_is_bounded_when_signer_stops_responding(self):
    for i in range(SIGNER_MAX_PENDING):
      sends = self.update_control(1_000_000_000 + i * 10_000_000, angle=float(i))
      self.assertEqual(len(signer_request_frames(sends)), 4)

    self.assertEqual(len(self.transport.pending_requests), SIGNER_MAX_PENDING)
    self.assertFalse(self.transport.control_generation_due(
      enabled=True, lat_active=True, long_active=True, now_nanos=1_080_000_000,
    ))

    sends = self.update_control(1_080_000_000, angle=20.0)
    self.assertFalse(signer_request_frames(sends))

    sends = self.update_control(1_100_000_000, angle=20.0)
    self.assertEqual(self.transport.last_failure_reason, "signer_timeout")
    self.assertEqual(len(signer_request_frames(sends)), 4)
    self.assertEqual(self.transport.control_started_ns, 1_100_000_000)

  def test_disable_releases_request_plane(self):
    self.finish_handoff()
    sends = self.update_control(1_030_000_000, enabled=False, lat_active=False, long_active=False)
    self.assertIn(CanData(SIGNER_ADDR, bytes.fromhex("07c9a80000000000"), TSS3_AUX_BUS), sends)
    self.assertFalse(self.transport.active)

  def test_invalid_can_releases_request_plane(self):
    self.finish_handoff()
    self.transport.observe([], False)
    sends = self.update_control(1_030_000_000)
    self.assertIn(CanData(SIGNER_ADDR, bytes.fromhex("07c9a80000000000"), TSS3_AUX_BUS), sends)
    self.assertEqual(self.transport.last_failure_reason, "can_invalid")
    self.assertFalse(self.transport.active)

  def test_handoff_reject_restarts_acquisition(self):
    sequence = self.start_request()
    self.transport.observe(response(1_015_000_000, sequence), True)
    sends = self.update_control(1_020_000_000)
    host = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.transport.observe(packets(1_021_000_000, CanData(host.address, host.dat,
                                                         TSS3_CHASSIS_BUS + PANDA_REJECTED_OFFSET)), True)
    self.assertEqual(self.transport.last_failure_reason, "handoff_host_frame_rejected")
    sends = self.update_control(1_022_000_000)
    self.assertIn(CanData(SIGNER_ADDR, bytes.fromhex("07c9a80000000000"), TSS3_AUX_BUS), sends)
    self.assertEqual(len(signer_request_frames(sends)), 4)

  def test_handoff_reject_recovers_without_new_engagement(self):
    sequence = self.start_request()
    self.transport.observe(response(1_015_000_000, sequence), True)
    sends = self.update_control(1_020_000_000)
    host = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.transport.observe(packets(1_021_000_000, CanData(host.address, host.dat,
                                                         TSS3_CHASSIS_BUS + PANDA_REJECTED_OFFSET)), True)
    sends = self.update_control(1_022_000_000)
    retry_sequence = request_sequence_from_sends(sends)
    self.transport.observe(response(1_037_000_000, retry_sequence), True)
    sends = self.update_control(1_042_000_000)
    retry_host = next(msg for msg in sends if msg.address == CONTROL_REQUEST_ADDR)
    self.transport.observe(packets(1_042_000_001, CanData(retry_host.address, retry_host.dat,
                                                         TSS3_CHASSIS_BUS + PANDA_RETURNED_OFFSET)), True)
    self.assertTrue(self.transport.active)
    self.assertEqual(self.transport.last_failure_reason, "")


if __name__ == "__main__":
  unittest.main()
