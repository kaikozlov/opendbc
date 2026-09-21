"""TSS3 request construction and EPS signer transport helpers."""
from dataclasses import dataclass
from math import isfinite

from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog

TSS3_B6_TARGET_ANGLE_SCALE_DEG = 1024 / 17870
TSS3_SIGNER_CONTROL_ADDR = 0x777
TSS3_SIGNER_CONTROL_MAGIC = b"\x07\xC7\xC7"
TSS3_SIGNER_BUS = 1

TSS3_NO_LATERAL_REQUEST_ID = 0
TSS3_LTA_LCA_ID = 11
TSS3_LTA_ASSIST_GAIN_RAW = 100
TSS3_INACTIVE_ASSIST_GAIN_RAW = 50
TSS3_LONGITUDINAL_HOST_REQUEST = (0x2D, 0x47)
NATIVE_08A_ADDR = 0x08A
ADMIN_ADDR = 0x777
DOWNSTREAM_BUS = 0
ADMIN_BUS = 1
SOURCE_BUS = 2
PANDA_RETURNED_OFFSET = 0x80
PANDA_REJECTED_OFFSET = 0xC0

ORACLE_REQUEST_ADDR = 0x777
ORACLE_RESPONSE_ADDR = 0x7A9
ORACLE_BUS = 0
ORACLE_PRIVATE_SID = 0xC9
ORACLE_SEQUENCE_MAX = 0x1F
ORACLE_RETRY_TIMEOUT_NS = 50_000_000
ORACLE_PUBLICATION_DEADLINE_NS = 90_000_000


def target_angle_deg_to_raw(angle_deg: float) -> int:
  return int(round(angle_deg / TSS3_B6_TARGET_ANGLE_SCALE_DEG))


def build_signer_control(target_angle_raw: int, control_sequence: int) -> tuple[int, bytes, int]:
  """Build the bounded sideband consumed by a TSS3 EPS-resident signer.

  Zero is the neutral sequence. Active commands use 1..255 and wrap without
  passing through zero.
  """
  if not -(1 << 15) <= target_angle_raw < (1 << 15):
    raise ValueError("target angle must fit signed16")
  if not 0 <= control_sequence <= 0xFF:
    raise ValueError("signer control sequence must be 0..255")

  data = (TSS3_SIGNER_CONTROL_MAGIC + bytes((control_sequence,)) +
          target_angle_raw.to_bytes(2, "big", signed=True) + bytes(2))
  return TSS3_SIGNER_CONTROL_ADDR, data, TSS3_SIGNER_BUS


def build_host_application(packer, *, lat_active: bool, target_angle_raw: int,
                           long_active: bool, accel: float,
                           set_speed_kph: float, request_sequence: int) -> bytes:
  """Build the complete comma-owned TSS3 0x08A application.

  The FRC frame is never an application template or freshness source. The
  constants below are the dominant complete Camry active and inactive
  envelopes measured across 44,613 native publications.
  """
  if not -(1 << 15) <= target_angle_raw < (1 << 15):
    raise ValueError("target angle must fit signed16")
  if not 0 <= request_sequence <= 0x3F:
    raise ValueError("request sequence must fit u6")

  accel_request = float(accel) if long_active else 0.0
  if not -32.768 <= accel_request <= 32.767:
    raise ValueError("acceleration must fit signed16 at 0.001 m/s^2")
  set_speed = max(0, min(255, int(round(set_speed_kph)))) if isfinite(set_speed_kph) else 0

  values = {
    "CRUISE_OPERATING_LATCH": 1,
    "SET_ME_X80": 0x80,
    "LONGITUDINAL_REQUEST_ID_A": TSS3_LONGITUDINAL_HOST_REQUEST[0] >> 2,
    "LONGITUDINAL_ALLOCATION_METHOD_A": TSS3_LONGITUDINAL_HOST_REQUEST[0] & 0x3,
    "LONGITUDINAL_REQUEST_ID_B": TSS3_LONGITUDINAL_HOST_REQUEST[1] >> 2,
    "LONGITUDINAL_ALLOCATION_METHOD_B": TSS3_LONGITUDINAL_HOST_REQUEST[1] & 0x3,
    "LONGITUDINAL_REQUEST_ACCEL_A": accel_request,
    "SET_SPEED": set_speed,
    "LONGITUDINAL_REQUEST_ACCEL_B": accel_request,
    "SET_ME_X7FFF_A": 0x7FFF,
    "SET_ME_X7FFF_B": 0x7FFF,
    "LATERAL_REQUEST_PINION_ANGLE": target_angle_raw * 0.001000121519,
    "SET_ME_XC0": 0xC0,
    "LATERAL_REQUEST_ID": TSS3_LTA_LCA_ID if lat_active else TSS3_NO_LATERAL_REQUEST_ID,
    "SET_ME_X10": 0x10,
    "LATERAL_ASSIST_GAIN": (TSS3_LTA_ASSIST_GAIN_RAW if lat_active else TSS3_INACTIVE_ASSIST_GAIN_RAW) * 0.01,
    "REQUEST_SEQUENCE": request_sequence,
  }
  _, data, _ = packer.make_can_msg("TSS3_CONTROL_REQUEST", DOWNSTREAM_BUS, values)
  return data[:28]


def make_request_plane_admin(arm: bool) -> CanData:
  return CanData(ADMIN_ADDR, bytes((7, 0xC9, 0xA8, int(arm), 0, 0, 0, 0)), ADMIN_BUS)


def build_oracle_transport(seq: int, application: bytes) -> list[CanData]:
  """Carry one 28-byte application through the EPS raw diagnostic ring."""
  if not 1 <= seq <= ORACLE_SEQUENCE_MAX:
    raise ValueError(f"oracle sequence must be 1..{ORACLE_SEQUENCE_MAX}")
  if len(application) != 28:
    raise ValueError("oracle application must be exactly 28 bytes")

  stream = application + b"\0\0"
  return [CanData(ORACLE_REQUEST_ADDR,
                  bytes((0xC8, (fragment << 5) | seq)) + stream[fragment * 5:(fragment + 1) * 5] + b"\0",
                  ORACLE_BUS) for fragment in range(6)]


@dataclass
class SignRequest:
  sequence: int
  application: bytes
  control_epoch: int
  generation_started_ns: int
  sent_ns: int


class ToyotaTss3RequestTransport:
  """Single-flight transport for the latest normal CarController output.

  Native 0x08A arrivals provide publication cadence. Multiple arrivals while a
  signature is in flight coalesce into one next publication; actuator commands
  are never queued. Engagement and lat/long activity edges invalidate any
  unfinished or not-yet-transmitted application.
  """

  def __init__(self, packer):
    self.packer = packer
    self.can_valid = False
    self.control_enabled = False
    self.control_lat_active = False
    self.control_target_angle_raw = 0
    self.control_long_active = False
    self.control_accel = 0.0
    self.control_set_speed_kph = 0.0
    self.control_epoch = 0

    self.next_oracle_sequence = 1
    self.next_request_sequence = 0
    self.inflight: SignRequest | None = None
    self.publication_due = False
    self.publication_due_since_ns = 0
    self.ready_host_frame: tuple[bytes, int] | None = None
    self.pending_sends: list[CanData] = []

    self.active = False
    self.arm_pending = False
    self.arm_host_frame: bytes | None = None
    self.last_failure_reason = ""

  def authority_unavailable(self) -> bool:
    acquiring = self.arm_pending or self.inflight is not None or self.ready_host_frame is not None or self.publication_due
    return self.control_enabled and not self.active and not acquiring

  def _record_failure(self, reason: str) -> None:
    self.last_failure_reason = reason
    carlog.error(f"Toyota F33 request plane failure: {reason}")

  def _invalidate_actuation(self) -> None:
    self.inflight = None
    self.ready_host_frame = None

  def _release(self) -> None:
    if self.active or self.arm_pending:
      self.pending_sends.append(make_request_plane_admin(False))
    self.active = False
    self.arm_pending = False
    self.arm_host_frame = None
    self.publication_due = False
    self.publication_due_since_ns = 0
    self._invalidate_actuation()

  def _authority_failure(self, reason: str) -> None:
    self._record_failure(reason)
    self._release()

  def _mark_publication_due(self, now_ns: int) -> None:
    if not self.publication_due:
      self.publication_due_since_ns = now_ns
    self.publication_due = True

  def _retry_latest(self, now_ns: int, reason: str) -> None:
    if self.inflight is None:
      return
    generation_started_ns = self.inflight.generation_started_ns
    self.inflight = None
    if now_ns - generation_started_ns > ORACLE_PUBLICATION_DEADLINE_NS:
      self._authority_failure("oracle_dead")
      return
    self.publication_due = True
    self.publication_due_since_ns = generation_started_ns
    carlog.warning(f"Toyota F33 request plane retrying latest control: {reason}")

  def _observe_tx_echo(self, address: int, data: bytes, src: int) -> None:
    if address == ADMIN_ADDR and self.arm_pending and data == make_request_plane_admin(True).dat:
      if src == ADMIN_BUS + PANDA_REJECTED_OFFSET:
        self._authority_failure("arm_admin_rejected")
      return
    if address != NATIVE_08A_ADDR:
      return
    if src == DOWNSTREAM_BUS + PANDA_RETURNED_OFFSET and self.arm_pending and data == self.arm_host_frame:
      self.active = True
      self.arm_pending = False
      self.arm_host_frame = None
    elif src == DOWNSTREAM_BUS + PANDA_REJECTED_OFFSET and self.arm_pending and data == self.arm_host_frame:
      self._authority_failure("handoff_host_frame_rejected")
    elif src == DOWNSTREAM_BUS + PANDA_REJECTED_OFFSET and self.active:
      carlog.warning("Toyota F33 request plane TX rejected")

  def _observe_oracle_response(self, data: bytes, now_ns: int) -> None:
    if len(data) != 8 or data[0] != ORACLE_PRIVATE_SID:
      return
    seq, status = data[1], data[2]
    if not 1 <= seq <= ORACLE_SEQUENCE_MAX or data[3] != (seq ^ 0xFF):
      return
    if self.inflight is None or seq != self.inflight.sequence:
      return
    if now_ns - self.inflight.generation_started_ns > ORACLE_PUBLICATION_DEADLINE_NS:
      self._authority_failure("oracle_dead")
      return
    if status != 0:
      self._retry_latest(now_ns, "oracle_sign_status")
      return

    request = self.inflight
    self.inflight = None
    self.ready_host_frame = (request.application + data[4:8], request.control_epoch)

  def observe(self, can_packets: list[tuple[int, list[CanData]]], can_valid: bool) -> None:
    self.can_valid = bool(can_valid)
    if not self.can_valid and (self.active or self.arm_pending):
      self._authority_failure("can_invalid")

    for nanos, packets in can_packets:
      for address, data, src in packets:
        address_i, src_i, payload = int(address), int(src), bytes(data)
        if src_i >= PANDA_RETURNED_OFFSET:
          self._observe_tx_echo(address_i, payload, src_i)
        elif src_i == ORACLE_BUS and address_i == ORACLE_RESPONSE_ADDR:
          self._observe_oracle_response(payload, int(nanos))
        elif src_i == SOURCE_BUS and address_i == NATIVE_08A_ADDR and len(payload) == 32:
          self._mark_publication_due(int(nanos))

  def _expire(self, now_ns: int) -> None:
    if self.inflight is None:
      return
    if now_ns - self.inflight.generation_started_ns > ORACLE_PUBLICATION_DEADLINE_NS:
      self._authority_failure("oracle_dead")
    elif now_ns - self.inflight.sent_ns > ORACLE_RETRY_TIMEOUT_NS:
      self._retry_latest(now_ns, "oracle_response_timeout")

  def _start_signing_latest(self, now_ns: int) -> None:
    if not self.control_enabled or not self.can_valid or not self.publication_due or self.inflight is not None:
      return

    application = build_host_application(
      self.packer,
      lat_active=self.control_lat_active,
      target_angle_raw=self.control_target_angle_raw,
      long_active=self.control_long_active,
      accel=self.control_accel,
      set_speed_kph=self.control_set_speed_kph,
      request_sequence=self.next_request_sequence,
    )
    seq = self.next_oracle_sequence
    self.next_oracle_sequence = (seq % ORACLE_SEQUENCE_MAX) + 1
    self.next_request_sequence = (self.next_request_sequence + 1) & 0x3F
    generation_started_ns = self.publication_due_since_ns or now_ns
    self.publication_due = False
    self.publication_due_since_ns = 0
    self.inflight = SignRequest(seq, application, self.control_epoch, generation_started_ns, now_ns)
    self.pending_sends.extend(build_oracle_transport(seq, application))

  def update_control(self, *, enabled: bool, lat_active: bool, target_angle_deg: float,
                     long_active: bool, accel: float, set_speed_kph: float,
                     now_nanos: int) -> list[CanData]:
    enabled = bool(enabled)
    lat_active = enabled and bool(lat_active)
    long_active = enabled and bool(long_active)
    control_state = (enabled, lat_active, long_active)
    previous_state = (self.control_enabled, self.control_lat_active, self.control_long_active)
    if control_state != previous_state:
      self.control_epoch += 1
      self._invalidate_actuation()
      if enabled and not self.control_enabled:
        self.next_request_sequence = 0

    self.control_enabled = enabled
    self.control_lat_active = lat_active
    self.control_target_angle_raw = target_angle_deg_to_raw(float(target_angle_deg))
    self.control_long_active = long_active
    self.control_accel = float(accel) if long_active else 0.0
    self.control_set_speed_kph = float(set_speed_kph)

    if not enabled:
      self._release()
    else:
      self._expire(now_nanos)

      if self.ready_host_frame is not None:
        frame, epoch = self.ready_host_frame
        self.ready_host_frame = None
        if epoch == self.control_epoch:
          if not self.active and not self.arm_pending:
            # Keep stock publication live while the EPS signs. Switch ownership
            # only when the first replacement is ready, with admin immediately
            # followed by that frame in the ordinary CarController send batch.
            self.pending_sends.append(make_request_plane_admin(True))
            self.arm_pending = True
            self.arm_host_frame = frame
          self.pending_sends.append(CanData(NATIVE_08A_ADDR, frame, DOWNSTREAM_BUS))

      self._start_signing_latest(now_nanos)

    sends, self.pending_sends = self.pending_sends, []
    return sends


# Kept as source-compatible aliases for the exact-F33 analysis tooling.
build_f33_signer_control = build_signer_control
TSS3_F33_SIGNER_CONTROL_ADDR = TSS3_SIGNER_CONTROL_ADDR
TSS3_F33_SIGNER_CONTROL_MAGIC = TSS3_SIGNER_CONTROL_MAGIC
TSS3_F33_SIGNER_BUS = TSS3_SIGNER_BUS
