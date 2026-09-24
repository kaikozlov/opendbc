"""CONTROL_REQUEST (0x08A) construction and the EPS-resident SecOC signer transport."""
from collections import deque
from dataclasses import dataclass

from opendbc.car import DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.toyota.values import CarControllerParams

# The camera harness is repinned so the vehicle network is on bus 0 and the FRC on bus 2
TSS3_CHASSIS_BUS = 0
TSS3_AUX_BUS = 1
TSS3_SOURCE_BUS = 2

CONTROL_REQUEST_ADDR = 0x08A
SIGNER_ADDR = 0x777
SIGNER_RESPONSE_ADDR = 0x7A9
SIGNER_SID = 0xC9
SIGNER_SEQUENCE_MAX = 0xFF
# The signer answers in 17 ms (p50) to 32 ms (p99). Four requests in flight hide that at 100 Hz,
# more only add latency between computing a request and publishing it.
SIGNER_MAX_PENDING = 4
SIGNER_TIMEOUT_NS = 90_000_000
GENERATION_INTERVAL_NS = int(DT_CTRL * 1e9)

# The VMC in the brake ECU sets CONTROL_RESULT.REQUEST_LOSS ~90 ms after the last valid CONTROL_REQUEST and latches
# a cruise fault until restart if it persists for ~1 s. Panda forwards the FRC's CONTROL_REQUEST again if openpilot's
# stops for 100 ms, and it restarts the angle rate limit from the measured angle after a 100 ms gap in requests.
CONTROL_REQUEST_TIMEOUT_NS = 100_000_000

PANDA_RETURNED_OFFSET = 0x80
PANDA_REJECTED_OFFSET = 0xC0

LTA_LCA_REQUEST_ID = 11


def target_angle_deg_to_raw(angle_deg: float) -> int:
  return round(angle_deg / CarControllerParams.TSS3_TARGET_ANGLE_SCALE_DEG)


def build_host_application(packer, *, lat_active: bool, target_angle_raw: int,
                           long_active: bool, accel: float,
                           set_speed_kph: float, request_sequence: int,
                           stock_application: bytes | None = None) -> bytes:
  """Build the 28-byte CONTROL_REQUEST payload that the EPS signs.

  With stock longitudinal, only the lateral fields and sequence of the FRC's request are replaced.
  """
  accel = accel if long_active else 0.0
  values = {
    "CRUISE_OPERATING_LATCH": 1,
    "SET_ME_1": 1,
    "LONGITUDINAL_REQUEST_ID_UPPER": 11,
    "LONGITUDINAL_ALLOCATION_METHOD_UPPER": 1,  # engine and brake
    "LONGITUDINAL_REQUEST_ID_LOWER": 17,
    "LONGITUDINAL_ALLOCATION_METHOD_LOWER": 3,  # brake only
    "LONGITUDINAL_REQUEST_ACCEL_UPPER": accel,
    "LONGITUDINAL_REQUEST_ACCEL_LOWER": accel,
    "SET_SPEED": min(max(round(set_speed_kph), 0), 255),
    "SET_ME_X7FFF": 0x7FFF,
    "SET_ME_X7FFF_2": 0x7FFF,
    "LATERAL_REQUEST_PINION_ANGLE": target_angle_raw * 0.001000121519,
    "CRUISE_STATE_MIRROR": 3,
    "LATERAL_REQUEST_ID": LTA_LCA_REQUEST_ID if lat_active else 0,
    "CRUISE_REQUEST_ACTIVE": 1,
    "LATERAL_ASSIST_GAIN": 1.0 if lat_active else 0.5,
    "REQUEST_SEQUENCE": request_sequence,
  }
  _, data, _ = packer.make_can_msg("CONTROL_REQUEST", TSS3_CHASSIS_BUS, values)
  application = bytearray(stock_application if stock_application is not None else data[:28])

  if stock_application is not None:
    # lateral pinion angle, lateral request ID, assist gain, and sequence
    application[18:20] = data[18:20]
    application[21] = (application[21] & 0xC0) | (data[21] & 0x3F)
    application[24:26] = data[24:26]
    application[26] = (application[26] & 0xC0) | (request_sequence & 0x3F)

  return bytes(application)


def make_admin_msg(arm: bool) -> CanData:
  return CanData(SIGNER_ADDR, bytes((7, 0xC9, 0xA8, int(arm), 0, 0, 0, 0)), TSS3_AUX_BUS)


def build_signer_requests(seq: int, application: bytes) -> list[CanData]:
  # four 7-byte fragments, each header carries the fragment index and a nibble of the sequence
  low, high = seq & 0x0F, seq >> 4
  headers = (0x80 | low, 0x90 | high, 0xA0 | low, 0xB0 | high)
  return [CanData(SIGNER_ADDR,
                  bytes((headers[fragment],)) + application[fragment * 7:(fragment + 1) * 7],
                  TSS3_CHASSIS_BUS) for fragment in range(4)]


@dataclass
class SignRequest:
  sequence: int
  application: bytes
  control_epoch: int
  generation_started_ns: int
  trailer: bytes | None = None
  failed: bool = False
  superseded: bool = False


class ToyotaTss3RequestTransport:
  """Signs one CONTROL_REQUEST per controller frame through the EPS and publishes the signed result.

  The EPS returns the SecOC freshness value and MAC. A few requests are kept in flight to hide the signer latency.
  Panda checks each request when it is sent to the signer, and only lets approved requests be published.
  """

  def __init__(self, packer, *, stock_longitudinal: bool = False):
    self.packer = packer
    self.stock_longitudinal = stock_longitudinal
    self.stock_application: bytes | None = None
    self.can_valid = False
    self.control_enabled = False
    self.control_lat_active = False
    self.control_target_angle_raw = 0
    self.control_long_active = False
    self.control_accel = 0.0
    self.control_set_speed_kph = 0.0
    self.control_epoch = 0
    self.control_started_ns = 0

    self.next_signer_sequence = 1
    self.next_request_sequence = 0
    self.last_request_ns = 0
    self.request_rejected = False
    self.pending_requests: deque[SignRequest] = deque()
    self.requests_by_sequence: dict[int, SignRequest] = {}
    self.pending_sends: list[CanData] = []

    self.active = False
    self.arm_pending = False
    self.arm_host_frame: bytes | None = None
    self.last_publication_ns = 0
    self.last_host_send_ns = 0
    self.last_host_request_sequence: int | None = None
    self.last_failure_reason = ""

  def _record_failure(self, reason: str) -> None:
    self.last_failure_reason = reason
    carlog.error(f"Toyota TSS3 request plane failure: {reason}")

  def _remove_request(self, request: SignRequest) -> None:
    if self.requests_by_sequence.get(request.sequence) is request:
      del self.requests_by_sequence[request.sequence]

  def _prune_head(self) -> None:
    while self.pending_requests:
      request = self.pending_requests[0]
      if request.control_epoch == self.control_epoch and not request.failed and not request.superseded:
        break
      self.pending_requests.popleft()
      self._remove_request(request)

  def _invalidate_actuation(self) -> None:
    self.pending_requests.clear()
    self.requests_by_sequence.clear()

  def _release(self) -> None:
    if self.active or self.arm_pending:
      self.pending_sends.append(make_admin_msg(False))
    self.active = False
    self.arm_pending = False
    self.arm_host_frame = None
    self.last_publication_ns = 0
    self.last_host_send_ns = 0
    self.last_host_request_sequence = None
    self.control_started_ns = 0
    self._invalidate_actuation()

  def _restart_after_failure(self, reason: str) -> None:
    self._record_failure(reason)
    self._release()

  def _observe_tx_echo(self, address: int, data: bytes, src: int, now_ns: int) -> None:
    # panda checks requests on the last fragment
    if address == SIGNER_ADDR and src == TSS3_CHASSIS_BUS + PANDA_REJECTED_OFFSET and (data[0] >> 4) == 0xB:
      self.request_rejected = True
      return
    if address == SIGNER_ADDR and self.arm_pending and data == make_admin_msg(True).dat:
      if src == TSS3_AUX_BUS + PANDA_REJECTED_OFFSET:
        self._restart_after_failure("arm_admin_rejected")
      return
    if address != CONTROL_REQUEST_ADDR:
      return

    if src == TSS3_CHASSIS_BUS + PANDA_RETURNED_OFFSET:
      if self.arm_pending and data == self.arm_host_frame:
        self.active = True
        self.arm_pending = False
        self.arm_host_frame = None
        self.last_publication_ns = now_ns
        self.last_failure_reason = ""
      elif self.active:
        self.last_publication_ns = now_ns
    elif src == TSS3_CHASSIS_BUS + PANDA_REJECTED_OFFSET:
      # panda forwards the FRC's CONTROL_REQUEST again after a rejection, re-arm with the next signed frame
      if self.arm_pending and data == self.arm_host_frame:
        self._restart_after_failure("handoff_host_frame_rejected")
      elif self.active:
        self._restart_after_failure("host_frame_rejected")

  def _observe_signer_response(self, data: bytes, now_ns: int) -> None:
    if len(data) != 8 or data[0] != SIGNER_SID:
      return
    seq, status = data[1], data[2]
    if not 1 <= seq <= SIGNER_SEQUENCE_MAX or data[3] != (seq ^ 0xFF):
      return

    request = self.requests_by_sequence.get(seq)
    if request is None or request.control_epoch != self.control_epoch:
      return
    if now_ns - request.generation_started_ns > SIGNER_TIMEOUT_NS:
      request.superseded = True
      return

    # the signer answers in order, so earlier unanswered requests were lost
    for older in self.pending_requests:
      if older is request:
        break
      if older.trailer is None:
        older.superseded = True

    if status != 0:
      request.failed = True
      carlog.warning(f"Toyota TSS3 request plane signer status {status}")
    else:
      request.trailer = data[4:8]
    self._prune_head()

  def observe(self, can_packets: list[tuple[int, list[CanData]]], can_valid: bool) -> None:
    self.can_valid = bool(can_valid)
    if not self.can_valid:
      self.stock_application = None
    if not self.can_valid and (self.active or self.arm_pending):
      self._restart_after_failure("can_invalid")

    for nanos, packets in can_packets:
      now_ns = int(nanos)
      for address, data, src in packets:
        address_i, src_i, payload = int(address), int(src), bytes(data)
        if src_i >= PANDA_RETURNED_OFFSET:
          self._observe_tx_echo(address_i, payload, src_i, now_ns)
        elif src_i == TSS3_CHASSIS_BUS and address_i == SIGNER_RESPONSE_ADDR:
          self._observe_signer_response(payload, now_ns)
        elif src_i == TSS3_SOURCE_BUS and address_i == CONTROL_REQUEST_ADDR and len(payload) == 32:
          self.stock_application = payload[:28]

  def _expire(self, now_ns: int) -> None:
    self._prune_head()
    if not self.control_enabled or not self.can_valid:
      return

    reference_ns = self.last_publication_ns if self.active else self.control_started_ns
    if reference_ns and now_ns - reference_ns > SIGNER_TIMEOUT_NS:
      self._restart_after_failure("signer_timeout")

  def _head_ready_for_publication(self, now_ns: int) -> bool:
    self._prune_head()
    if not self.pending_requests:
      return False
    request = self.pending_requests[0]
    if request.trailer is None:
      return False

    request_sequence = request.application[26] & 0x3F
    if self.last_host_request_sequence is not None:
      generation_delta = (request_sequence - self.last_host_request_sequence) & 0x3F
      # keep 10ms per generation after a lost response, don't compress steps
      if generation_delta > 1 and now_ns - self.last_host_send_ns < generation_delta * GENERATION_INTERVAL_NS:
        return False
    return True

  def _take_ready_host_frame(self, now_ns: int) -> bytes | None:
    if not self._head_ready_for_publication(now_ns):
      return None

    request = self.pending_requests[0]
    request_sequence = request.application[26] & 0x3F
    self.pending_requests.popleft()
    self._remove_request(request)
    if request.control_epoch != self.control_epoch:
      return None
    self.last_host_send_ns = now_ns
    self.last_host_request_sequence = request_sequence
    return request.application + request.trailer

  def _allocate_signer_sequence(self) -> int | None:
    for _ in range(SIGNER_SEQUENCE_MAX):
      seq = self.next_signer_sequence
      self.next_signer_sequence = (seq % SIGNER_SEQUENCE_MAX) + 1
      if seq not in self.requests_by_sequence:
        return seq
    return None

  def _queue_signing_latest(self, now_ns: int) -> None:
    self._prune_head()
    if (not self.control_enabled or not self.can_valid or
        len(self.pending_requests) >= SIGNER_MAX_PENDING or
        (self.stock_longitudinal and self.stock_application is None)):
      return

    seq = self._allocate_signer_sequence()
    if seq is None:
      return
    if self.control_started_ns == 0:
      self.control_started_ns = now_ns
    application = build_host_application(
      self.packer,
      lat_active=self.control_lat_active,
      target_angle_raw=self.control_target_angle_raw,
      long_active=self.control_long_active,
      accel=self.control_accel,
      set_speed_kph=self.control_set_speed_kph,
      request_sequence=self.next_request_sequence,
      stock_application=self.stock_application if self.stock_longitudinal else None,
    )
    self.next_request_sequence = (self.next_request_sequence + 1) & 0x3F
    self.last_request_ns = now_ns
    request = SignRequest(seq, application, self.control_epoch, now_ns)
    self.pending_requests.append(request)
    self.requests_by_sequence[seq] = request
    self.pending_sends.extend(build_signer_requests(seq, application))

  def angle_reference_reset(self, now_nanos: int) -> bool:
    """Whether panda restarted its angle rate limit from the measured angle, the controller should do the same."""
    reset = self.request_rejected or now_nanos - self.last_request_ns > CONTROL_REQUEST_TIMEOUT_NS
    self.request_rejected = False
    return reset

  def control_generation_due(self, *, enabled: bool, lat_active: bool, long_active: bool,
                             now_nanos: int) -> bool:
    enabled = bool(enabled)
    if not enabled or not self.can_valid:
      return False
    if self.stock_longitudinal and self.stock_application is None:
      return False
    if not self.control_enabled:
      return True

    self._prune_head()

    front_will_publish = not self.arm_pending and self._head_ready_for_publication(now_nanos)
    pending_after_publication = len(self.pending_requests) - int(front_will_publish)
    return pending_after_publication < SIGNER_MAX_PENDING

  def update_control(self, *, enabled: bool, lat_active: bool, target_angle_deg: float,
                     long_active: bool, accel: float, set_speed_kph: float,
                     now_nanos: int) -> list[CanData]:
    enabled = bool(enabled)
    lat_active = enabled and bool(lat_active)
    long_active = enabled and bool(long_active)
    if enabled != self.control_enabled:
      self.control_epoch += 1
      self._invalidate_actuation()
      if enabled and not self.control_enabled:
        self.last_failure_reason = ""
        self.next_request_sequence = 0
        self.control_started_ns = 0
        self.last_publication_ns = 0

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
      # wait for panda to accept the first frame, then publish at most one frame per tick
      if not self.arm_pending:
        frame = self._take_ready_host_frame(now_nanos)
        if frame is not None:
          if not self.active:
            self.pending_sends.append(make_admin_msg(True))
            self.arm_pending = True
            self.arm_host_frame = frame
          self.pending_sends.append(CanData(CONTROL_REQUEST_ADDR, frame, TSS3_CHASSIS_BUS))

      self._queue_signing_latest(now_nanos)

    sends, self.pending_sends = self.pending_sends, []
    return sends
