"""Toyota TSS3 lateral analysis and receiver contracts.

The helpers in this module encode the target-native 2026 Camry/F33 protected-B6
receiver contract. Production output remains disabled. Historical Gate-2 sender
helpers are retained only for deterministic analysis/tests; no CarInterface or
CarController runtime path selects them after the upstream 0x08A request recovery.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Protocol
import struct
import time


TSS3_B6_ADDR = 0x0B6
TSS3_B6_LEN = 32
TSS3_B6_APPLICATION_LEN = 28
TSS3_B6_AUTH_INPUT_LEN = 36
TSS3_B6_TARGET_LATERAL_ID_INACTIVE = 0
TSS3_B6_TARGET_LATERAL_ID_LTA_LCA = 11
TSS3_B6_TARGET_ANGLE_SCALE_DEG = 1024 / 17870
TSS3_B6_TARGET_ANGLE_MAX_RAW = 1745
TSS3_B6_TARGET_DELTA_MAX_PER_GAP_RAW = 78
TSS3_B6_SEQUENCE_MODULUS = 64
TSS3_B6_EFFECTIVE_GAP_MAX = 8
TSS3_B6_RX_TIMEOUT_NS = 35_000_000
TSS3_STEERING_ANGLE_VELOCITY_MAX_RAW = 100

# Exact F33 0x394 classifier table projection. The EPS state table at CodeFlash
# 0x2A19C has five columns; 0x394 carries columns (4, 1, 2, 3). Two wire
# tuples are intentionally lossy, so callers receive candidate sets rather than
# a fabricated unique internal state. This is OEM-internal status, not an
# openpilot temporary/permanent fault policy.
TSS3_EPS_394_STATE_CANDIDATES: dict[tuple[int, int, int, int], tuple[int, ...]] = {
  (0, 0, 0, 0): (0,),
  (0, 1, 0, 0): (5,),
  (0, 2, 0, 0): (15,),
  (0, 3, 0, 0): (1, 3, 4),
  (0, 3, 2, 1): (7,),
  (0, 3, 3, 0): (9,),
  (0, 7, 0, 0): (2, 16),
  (1, 7, 1, 1): (10,),
  (1, 7, 4, 1): (11,),
  (1, 7, 5, 0): (14,),
  (1, 7, 6, 0): (13,),
  (1, 7, 7, 0): (12,),
  (2, 3, 2, 1): (6,),
  (2, 3, 3, 0): (8,),
}


def decode_eps_394_state_candidates(projection: tuple[int, int, int, int]) -> tuple[int, ...]:
  """Return exact-F33 internal classifier states compatible with 0x394.

  An empty tuple means the wire tuple is not present in the target-native
  17-row table. It is deliberately not converted into an openpilot fault.
  """
  return TSS3_EPS_394_STATE_CANDIDATES.get(projection, ())


@dataclass(frozen=True)
class TSS3B6Template:
  """Explicit 28-byte application template for fields not sender-closed yet.

  `stock_validated=False` is the only default.  The zero candidate is useful for
  deterministic packing/tests, but it must not be mistaken for a stock template.
  """

  application: bytes = bytes(TSS3_B6_APPLICATION_LEN)
  stock_validated: bool = False
  provenance: str = "explicit-zero-candidate; no stock B6 observed in retained factory-LTA intervals"

  def __post_init__(self):
    if len(self.application) != TSS3_B6_APPLICATION_LEN:
      raise ValueError(f"B6 application template must be {TSS3_B6_APPLICATION_LEN} bytes")


@dataclass(frozen=True)
class TSS3B6CompanionFields:
  """Configured scalar B6 companion fields with bounded names only."""

  signal263_b6_bit7: int = 0
  signal264_b6_bits6_4: int = 0
  # F33 proves value 1 suppresses one recovered additive steering term.
  additive_term_suppress: int = 1
  signal266_b6_bits1_0: int = 0
  signal267_b7_bits7_6: int = 0
  # F33 proves these are /100 contribution terms; zero removes each term.
  contribution_pct_1: int = 0
  contribution_pct_2: int = 0
  signal271_b10_bit7: int = 0
  signal272_b10_bit5: int = 0
  signal273_b10_bits2_0: int = 0

  def __post_init__(self):
    limits = {
      "signal263_b6_bit7": (self.signal263_b6_bit7, 1),
      "signal264_b6_bits6_4": (self.signal264_b6_bits6_4, 7),
      "additive_term_suppress": (self.additive_term_suppress, 1),
      "signal266_b6_bits1_0": (self.signal266_b6_bits1_0, 3),
      "signal267_b7_bits7_6": (self.signal267_b7_bits7_6, 3),
      "contribution_pct_1": (self.contribution_pct_1, 255),
      "contribution_pct_2": (self.contribution_pct_2, 255),
      "signal271_b10_bit7": (self.signal271_b10_bit7, 1),
      "signal272_b10_bit5": (self.signal272_b10_bit5, 1),
      "signal273_b10_bits2_0": (self.signal273_b10_bits2_0, 7),
    }
    for name, (value, maximum) in limits.items():
      if not 0 <= value <= maximum:
        raise ValueError(f"{name} out of range: {value}")


@dataclass(frozen=True)
class TSS3B6Application:
  data: bytes
  target_lateral_id: int
  target_angle_raw: int
  sequence: int
  template_stock_validated: bool

  @property
  def target_angle_deg(self) -> float:
    return self.target_angle_raw * TSS3_B6_TARGET_ANGLE_SCALE_DEG


@dataclass(frozen=True)
class TSS3Freshness:
  trip_counter: int
  reset_counter: int
  message_counter: int

  @property
  def reset_low2(self) -> int:
    return self.reset_counter & 0x3

  @property
  def message_low2(self) -> int:
    return self.message_counter & 0x3

  def full_bytes(self) -> bytes:
    if not 0 <= self.trip_counter < (1 << 16):
      raise ValueError("trip counter must be 16-bit")
    if not 0 <= self.reset_counter < (1 << 20):
      raise ValueError("reset counter must be 20-bit")
    if not 0 <= self.message_counter < (1 << 8):
      raise ValueError("message counter must be 8-bit")

    # FV46 carried in a 48-bit buffer:
    # trip16 || reset20 || message8 || reset_low2 || 00b.
    tail = (self.reset_counter << 12) | (self.message_counter << 4) | (self.reset_low2 << 2)
    return struct.pack(">HI", self.trip_counter, tail)

  @property
  def transmitted_nibble(self) -> int:
    # B28[7:4] = message_low2 || reset_low2.
    return (self.message_low2 << 2) | self.reset_low2


@dataclass
class TSS3ReplacementFreshnessState:
  """Conservative replacement-sender freshness state.

  The first authenticated 0x00F only establishes a baseline.  A replacement
  sender that starts without the previous B6 message8 waits for a *strictly
  newer* authenticated sync epoch, then owns message8 locally.  Application
  sequence is independent and advances modulo 64.
  """

  baseline_epoch: int | None = None
  active_trip: int | None = None
  active_reset: int | None = None
  message_counter: int | None = None
  application_sequence: int = 0
  armed: bool = False

  @staticmethod
  def _epoch(trip_counter: int, reset_counter: int) -> int:
    if not 0 <= trip_counter < (1 << 16) or not 0 <= reset_counter < (1 << 20):
      raise ValueError("invalid 0x00F trip/reset width")
    return (trip_counter << 20) | reset_counter

  @staticmethod
  def _strictly_newer(new_epoch: int, old_epoch: int) -> bool:
    # Conservative modulo-36 comparison; values exactly half a range apart are
    # intentionally not considered ordered.
    modulus = 1 << 36
    delta = (new_epoch - old_epoch) & (modulus - 1)
    return 0 < delta < (modulus >> 1)

  def observe_sync(self, trip_counter: int, reset_counter: int, *, authenticated: bool) -> bool:
    if not authenticated:
      return False

    epoch = self._epoch(trip_counter, reset_counter)
    if self.baseline_epoch is None:
      self.baseline_epoch = epoch
      self.active_trip = trip_counter
      self.active_reset = reset_counter
      return False

    if not self._strictly_newer(epoch, self.baseline_epoch):
      return False

    self.baseline_epoch = epoch
    self.active_trip = trip_counter
    self.active_reset = reset_counter
    # Exact receiver new-epoch semantics permit the first replacement B6 to
    # seed full message8 from its transmitted low2.  Choose 1 deterministically;
    # this is replacement policy, not a claim about Toyota's stock start value.
    self.message_counter = 1
    self.application_sequence = 0
    self.armed = True
    return True

  def observe_sync_fields(self, sync: dict[str, float], authenticator: TSS3SyncAuthenticator) -> bool:
    """Parse and authenticate a decoded 0x00F synchronization frame.

    No implicit trust is assigned to bus traffic here. A future live transport
    may back `TSS3SyncAuthenticator` with the provisioned ICU-S verifier; tests
    can use a known-key implementation. This keeps freshness ownership separate
    from the command-5 generation interface.
    """
    trip_counter = int(sync["TRIP_CNT"])
    reset_counter = int(sync["RESET_CNT"])
    tag = int(sync["AUTHENTICATOR"])
    authenticated = authenticator.verify_sync(trip_counter=trip_counter, reset_counter=reset_counter, authenticator=tag)
    return self.observe_sync(trip_counter, reset_counter, authenticated=authenticated)

  def next(self) -> tuple[TSS3Freshness, int]:
    if not self.armed or self.active_trip is None or self.active_reset is None or self.message_counter is None:
      raise RuntimeError("replacement freshness is not anchored to a newer authenticated 0x00F epoch")

    freshness = TSS3Freshness(self.active_trip, self.active_reset, self.message_counter)
    sequence = self.application_sequence
    self.message_counter = (self.message_counter + 1) & 0xFF
    self.application_sequence = (self.application_sequence + 1) & 0x3F
    return freshness, sequence


@dataclass(frozen=True)
class TSS3SignerResult:
  cmac128: bytes
  status: int = 0

  def __post_init__(self):
    if len(self.cmac128) != 16:
      raise ValueError("command-5 signer must return a full 16-byte CMAC")


class TSS3Signer(Protocol):
  def sign_cmac128(self, auth_input: bytes) -> TSS3SignerResult: ...


class TSS3SyncAuthenticator(Protocol):
  def verify_sync(self, *, trip_counter: int, reset_counter: int, authenticator: int) -> bool: ...


@dataclass
class TSS3SignerStats:
  calls: int = 0
  successes: int = 0
  failures: int = 0
  last_latency_ns: int | None = None
  min_latency_ns: int | None = None
  max_latency_ns: int | None = None
  total_latency_ns: int = 0

  @property
  def mean_latency_ns(self) -> float | None:
    return self.total_latency_ns / self.calls if self.calls else None


class InstrumentedTSS3Signer:
  """Latency/status instrumentation around a future command-5 transport."""

  def __init__(self, signer: TSS3Signer, clock_ns: Callable[[], int] = time.perf_counter_ns):
    self.signer = signer
    self.clock_ns = clock_ns
    self.stats = TSS3SignerStats()

  def sign_cmac128(self, auth_input: bytes) -> TSS3SignerResult:
    if len(auth_input) != TSS3_B6_AUTH_INPUT_LEN:
      raise ValueError(f"B6 command-5 auth input must be {TSS3_B6_AUTH_INPUT_LEN} bytes")
    start = self.clock_ns()
    try:
      result = self.signer.sign_cmac128(auth_input)
      if result.status != 0:
        raise RuntimeError(f"command-5 signer status {result.status}")
      self.stats.successes += 1
      return result
    except Exception:
      self.stats.failures += 1
      raise
    finally:
      latency = self.clock_ns() - start
      self.stats.calls += 1
      self.stats.last_latency_ns = latency
      self.stats.total_latency_ns += latency
      self.stats.min_latency_ns = latency if self.stats.min_latency_ns is None else min(self.stats.min_latency_ns, latency)
      self.stats.max_latency_ns = latency if self.stats.max_latency_ns is None else max(self.stats.max_latency_ns, latency)


@dataclass(frozen=True)
class TSS3SignedB6:
  application: TSS3B6Application
  freshness: TSS3Freshness
  auth_input: bytes
  data: bytes


@dataclass(frozen=True)
class TSS3Gate2DevelopmentConfig:
  """Live-supplied gates for the exact-F33 persistent-bypass development path.

  Nothing in this object is inferred from static analysis. `template` and
  `cadence_frames` come from the relay-correct stock capture, while the two
  booleans record completed live causal/topology checks. Production code must
  use an authenticated signer instead.
  """

  template: TSS3B6Template
  cadence_frames: int
  gate2_bypass_validated: bool
  exclusive_b6_authority_validated: bool

  def __post_init__(self):
    if not self.template.stock_validated:
      raise ValueError("development B6 template must be stock-validated")
    if not 1 <= self.cadence_frames <= 3:
      raise ValueError("development B6 cadence must be 1..3 control frames (<=30 ms)")
    if not self.gate2_bypass_validated:
      raise ValueError("exact-F33 Gate-2 bypass must be live-validated")
    if not self.exclusive_b6_authority_validated:
      raise ValueError("exclusive B6 relay/source authority must be live-validated")


@dataclass(frozen=True)
class TSS3Gate2DevelopmentFrame:
  application: TSS3B6Application
  freshness: TSS3Freshness
  data: bytes
  safety: TSS3SafetyDecision


class TSS3Gate2DevelopmentSender:
  """Historical fail-closed invalid-MAC Gate-2 experiment helper.

  No production/runtime Toyota interface selects this class. It remains only to
  preserve deterministic B6 receiver/freshness experiments. Retained Camry
  factory LTA/LCA uses zero B6 and exact F33 has a B6-independent internal
  assist path; 0x08A producer/SecOC ownership is a separate network question.
  """

  def __init__(self, config: TSS3Gate2DevelopmentConfig):
    self.config = config
    self.freshness = TSS3ReplacementFreshnessState()
    self.safety = TSS3PandaSafetyCandidate()
    self.previous_target_raw: int | None = None
    self._inactive = True

  def observe_sync_unverified_for_gate2_development(self, sync: dict[str, float]) -> bool:
    return self.freshness.observe_sync(int(sync["TRIP_CNT"]), int(sync["RESET_CNT"]), authenticated=True)

  def note_inactive(self) -> None:
    if not self._inactive:
      self.freshness.armed = False
      self.safety = TSS3PandaSafetyCandidate()
    self._inactive = True

  def build_if_due(self, *, frame: int, desired_target_raw: int, steering_angle_velocity_raw: int, now_nanos: int,
                   companions: TSS3B6CompanionFields | None = None) -> TSS3Gate2DevelopmentFrame | None:
    if frame % self.config.cadence_frames != 0 or not self.freshness.armed:
      return None

    desired_target_raw = max(-TSS3_B6_TARGET_ANGLE_MAX_RAW, min(TSS3_B6_TARGET_ANGLE_MAX_RAW, desired_target_raw))
    if self.previous_target_raw is not None:
      desired_target_raw = max(self.previous_target_raw - TSS3_B6_TARGET_DELTA_MAX_PER_GAP_RAW,
                               min(self.previous_target_raw + TSS3_B6_TARGET_DELTA_MAX_PER_GAP_RAW, desired_target_raw))

    sequence = self.freshness.application_sequence
    decision = self.safety.check(
      target_lateral_id=TSS3_B6_TARGET_LATERAL_ID_LTA_LCA,
      target_angle_raw=desired_target_raw,
      sequence=sequence,
      steering_angle_velocity_raw=steering_angle_velocity_raw,
      now_nanos=now_nanos,
    )
    if not decision.static_limits_ok:
      return None

    freshness, actual_sequence = self.freshness.next()
    if actual_sequence != sequence:
      raise AssertionError("development freshness/application sequence drift")
    application = build_b6_application(
      target_lateral_id=TSS3_B6_TARGET_LATERAL_ID_LTA_LCA,
      target_angle_raw=desired_target_raw,
      sequence=sequence,
      template=self.config.template,
      companions=companions,
    )
    # Intentionally invalid MAC: this path exists only after exact-F33 Gate-2
    # bypass behavior has been causally validated live. Keep the FV4 nibble real.
    data = application.data + build_b6_trailer(freshness, bytes(16))
    self.previous_target_raw = desired_target_raw
    self._inactive = False
    return TSS3Gate2DevelopmentFrame(application, freshness, data, decision)


def target_angle_deg_to_raw(angle_deg: float) -> int:
  return int(round(angle_deg / TSS3_B6_TARGET_ANGLE_SCALE_DEG))


def build_b6_application(*, target_lateral_id: int, target_angle_raw: int, sequence: int,
                         template: TSS3B6Template, companions: TSS3B6CompanionFields | None = None) -> TSS3B6Application:
  companions = companions or TSS3B6CompanionFields()
  if not 0 <= target_lateral_id <= 0x3F:
    raise ValueError("Target Lateral ID must be 6-bit")
  if not -(1 << 15) <= target_angle_raw < (1 << 15):
    raise ValueError("target angle must fit signed16")
  if not 0 <= sequence < TSS3_B6_SEQUENCE_MODULUS:
    raise ValueError("application sequence must be modulo64")

  data = bytearray(template.application)
  # Preserve B3[7:6], which is outside scalar signal261.
  data[3] = (data[3] & 0xC0) | target_lateral_id
  data[4:6] = int(target_angle_raw).to_bytes(2, "big", signed=True)

  # Preserve B6 bit3, which is not one of the recovered scalar fields.
  data[6] = ((companions.signal263_b6_bit7 & 1) << 7) | \
            ((companions.signal264_b6_bits6_4 & 7) << 4) | \
            (data[6] & 0x08) | \
            ((companions.additive_term_suppress & 1) << 2) | \
            (companions.signal266_b6_bits1_0 & 3)
  data[7] = ((companions.signal267_b7_bits7_6 & 3) << 6) | sequence
  data[8] = companions.contribution_pct_1
  data[9] = companions.contribution_pct_2
  # Preserve B10 bits6,4,3: these are not recovered scalar fields.
  data[10] = ((companions.signal271_b10_bit7 & 1) << 7) | (data[10] & 0x58) | \
             ((companions.signal272_b10_bit5 & 1) << 5) | (companions.signal273_b10_bits2_0 & 7)

  return TSS3B6Application(bytes(data), target_lateral_id, target_angle_raw, sequence, template.stock_validated)


def build_b6_auth_input(application: bytes, freshness: TSS3Freshness) -> bytes:
  if len(application) != TSS3_B6_APPLICATION_LEN:
    raise ValueError("B6 application must be exactly 28 bytes")
  auth_input = struct.pack(">H", TSS3_B6_ADDR) + application + freshness.full_bytes()
  if len(auth_input) != TSS3_B6_AUTH_INPUT_LEN:
    raise AssertionError("internal B6 auth input length drift")
  return auth_input


def build_b6_trailer(freshness: TSS3Freshness, cmac128: bytes) -> bytes:
  if len(cmac128) != 16:
    raise ValueError("CMAC must be 16 bytes")
  # Equivalent to hex(flags)[0] || CMAC_MSB28, matching Toyota's four-byte
  # FV4/MAC28 envelope.
  return bytes.fromhex(f"{freshness.transmitted_nibble:x}{cmac128.hex()[:7]}")


def sign_b6(application: TSS3B6Application, freshness: TSS3Freshness, signer: TSS3Signer) -> TSS3SignedB6:
  auth_input = build_b6_auth_input(application.data, freshness)
  result = signer.sign_cmac128(auth_input)
  if result.status != 0:
    raise RuntimeError(f"command-5 signer status {result.status}")
  data = application.data + build_b6_trailer(freshness, result.cmac128)
  if len(data) != TSS3_B6_LEN:
    raise AssertionError("internal B6 payload length drift")
  return TSS3SignedB6(application, freshness, auth_input, data)


@dataclass(frozen=True)
class TSS3SafetyDecision:
  static_limits_ok: bool
  production_allowed: bool
  reasons: tuple[str, ...] = field(default_factory=tuple)


@dataclass
class TSS3PandaSafetyCandidate:
  """Shadow safety contract derived from F33, deliberately non-enabling.

  This mirrors the constraints a future Panda mode should enforce.  It returns
  `production_allowed=False` unconditionally until the live policy gates are
  closed and the constraints are moved into a selectable C safety mode.
  """

  previous_angle_raw: int | None = None
  previous_sequence: int | None = None
  previous_candidate_ns: int | None = None

  def check(self, *, target_lateral_id: int, target_angle_raw: int, sequence: int,
            steering_angle_velocity_raw: int, now_nanos: int) -> TSS3SafetyDecision:
    reasons: list[str] = []
    active = target_lateral_id != TSS3_B6_TARGET_LATERAL_ID_INACTIVE

    if active and target_lateral_id != TSS3_B6_TARGET_LATERAL_ID_LTA_LCA:
      reasons.append("active Target Lateral ID must be exact LTA/LCA value 11")
    if not active and target_angle_raw != 0:
      reasons.append("inactive request must carry zero target angle")
    if abs(target_angle_raw) > TSS3_B6_TARGET_ANGLE_MAX_RAW:
      reasons.append("target angle exceeds F33 absolute envelope")
    if abs(steering_angle_velocity_raw) > TSS3_STEERING_ANGLE_VELOCITY_MAX_RAW:
      reasons.append("steering angle velocity exceeds F33 monitor threshold")

    if self.previous_sequence is not None:
      gap = (sequence - self.previous_sequence) & 0x3F
      # A replacement sender deliberately chooses strict +1 progression even
      # though the EPS receiver can tolerate a larger capped gap.
      if gap != 1:
        reasons.append("replacement application sequence must advance exactly +1 modulo64")
      effective_gap = min(max(gap, 1), TSS3_B6_EFFECTIVE_GAP_MAX)
      if self.previous_angle_raw is not None and abs(target_angle_raw - self.previous_angle_raw) > TSS3_B6_TARGET_DELTA_MAX_PER_GAP_RAW * effective_gap:
        reasons.append("target delta exceeds F33 per-gap envelope")

    if active and self.previous_candidate_ns is not None and now_nanos - self.previous_candidate_ns > TSS3_B6_RX_TIMEOUT_NS:
      reasons.append("active candidate missed the nominal 35ms EPS receive window")

    ok = not reasons
    if ok:
      self.previous_angle_raw = target_angle_raw
      self.previous_sequence = sequence
      self.previous_candidate_ns = now_nanos

    # Deliberately false: driver override, Q-current response, status/fault
    # policy, stock template/cadence, signer permission/latency, and exclusive
    # relay authority are live gates, and CarParams remains SafetyModel.noOutput.
    return TSS3SafetyDecision(ok, False, tuple(reasons))
