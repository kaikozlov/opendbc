"""Toyota TSS3 protected B6 lateral-control message helpers."""

from __future__ import annotations

from dataclasses import dataclass
import struct

from opendbc.car.secoc import add_mac_to_payload


TSS3_B6_ADDR = 0x0B6
TSS3_B6_LEN = 32
TSS3_B6_APPLICATION_LEN = 28
TSS3_B6_TARGET_LATERAL_ID_INACTIVE = 0
TSS3_B6_TARGET_LATERAL_ID_LTA_LCA = 11
TSS3_B6_TARGET_ANGLE_SCALE_DEG = 1024 / 17870
TSS3_B6_SEQUENCE_MODULUS = 64


@dataclass(frozen=True)
class TSS3B6Template:
  application: bytes = bytes(TSS3_B6_APPLICATION_LEN)

  def __post_init__(self):
    if len(self.application) != TSS3_B6_APPLICATION_LEN:
      raise ValueError(f"B6 application template must be {TSS3_B6_APPLICATION_LEN} bytes")


@dataclass(frozen=True)
class TSS3B6CompanionFields:
  """Exact-F33 scalar companion fields used by the B6 LTA/LCA command."""

  signal263_b6_bit7: int = 0
  signal264_b6_bits6_4: int = 0
  additive_term_suppress: int = 1
  signal266_b6_bits1_0: int = 0
  signal267_b7_bits7_6: int = 0
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
      "contribution_pct_1": (self.contribution_pct_1, 100),
      "contribution_pct_2": (self.contribution_pct_2, 100),
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

  @property
  def transmitted_nibble(self) -> int:
    return (self.message_low2 << 2) | self.reset_low2

  def full_bytes(self) -> bytes:
    if not 0 <= self.trip_counter < (1 << 16):
      raise ValueError("trip counter must be 16-bit")
    if not 0 <= self.reset_counter < (1 << 20):
      raise ValueError("reset counter must be 20-bit")
    if not 0 <= self.message_counter < (1 << 8):
      raise ValueError("message counter must be 8-bit")

    tail = (self.reset_counter << 12) | (self.message_counter << 4) | (self.reset_low2 << 2)
    return struct.pack(">HI", self.trip_counter, tail)


def target_angle_deg_to_raw(angle_deg: float) -> int:
  return int(round(angle_deg / TSS3_B6_TARGET_ANGLE_SCALE_DEG))


def build_b6_application(*, target_lateral_id: int, target_angle_raw: int, sequence: int,
                         template: TSS3B6Template, companions: TSS3B6CompanionFields) -> TSS3B6Application:
  if not 0 <= target_lateral_id <= 0x3F:
    raise ValueError("Target Lateral ID must be 6-bit")
  if not -(1 << 15) <= target_angle_raw < (1 << 15):
    raise ValueError("target angle must fit signed16")
  if not 0 <= sequence < TSS3_B6_SEQUENCE_MODULUS:
    raise ValueError("application sequence must be modulo64")

  data = bytearray(template.application)
  data[3] = (data[3] & 0xC0) | target_lateral_id
  data[4:6] = int(target_angle_raw).to_bytes(2, "big", signed=True)
  data[6] = ((companions.signal263_b6_bit7 & 1) << 7) | \
            ((companions.signal264_b6_bits6_4 & 7) << 4) | \
            (data[6] & 0x08) | \
            ((companions.additive_term_suppress & 1) << 2) | \
            (companions.signal266_b6_bits1_0 & 3)
  data[7] = ((companions.signal267_b7_bits7_6 & 3) << 6) | sequence
  data[8] = companions.contribution_pct_1
  data[9] = companions.contribution_pct_2
  data[10] = ((companions.signal271_b10_bit7 & 1) << 7) | (data[10] & 0x58) | \
             ((companions.signal272_b10_bit5 & 1) << 5) | (companions.signal273_b10_bits2_0 & 7)

  return TSS3B6Application(bytes(data), target_lateral_id, target_angle_raw, sequence)


def build_b6_zero_marker_frame(application: TSS3B6Application, freshness: TSS3Freshness) -> tuple[int, bytes, int]:
  """Build B6 for exact-F33 with the installed Gate-2 development patch.

  Preserve the live transmitted FV4 nibble while forcing only the 28 MAC bits to zero.
  """
  if len(application.data) != TSS3_B6_APPLICATION_LEN:
    raise ValueError(f"B6 application must be {TSS3_B6_APPLICATION_LEN} bytes")
  data = application.data + bytes.fromhex(f"{freshness.transmitted_nibble:x}0000000")
  if len(data) != TSS3_B6_LEN:
    raise AssertionError("internal B6 payload length drift")
  return TSS3_B6_ADDR, data, 0

def build_b6_secoc_frame(key: bytes, application: TSS3B6Application, freshness: TSS3Freshness) -> tuple[int, bytes, int]:
  """Build a normal secured B6 frame for the Gate-2-patched exact-F33 EPS.

  The installed receiver patch bypasses the CMAC comparison, not SecOC framing.
  Keep the stock DataID/application/freshness/MSB28 construction and sign with
  the configured (dummy on the patched platform) SecOC key.
  """
  if len(application.data) != TSS3_B6_APPLICATION_LEN:
    raise ValueError(f"B6 application must be {TSS3_B6_APPLICATION_LEN} bytes")
  data = add_mac_to_payload(
    key, freshness.trip_counter, freshness.reset_counter, freshness.message_counter,
    TSS3_B6_ADDR, application.data,
  )
  if len(data) != TSS3_B6_LEN:
    raise AssertionError("internal B6 payload length drift")
  return TSS3_B6_ADDR, data, 0
