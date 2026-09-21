"""TSS3 EPS-resident signer sideband helpers."""
from math import isfinite

TSS3_B6_TARGET_ANGLE_SCALE_DEG = 1024 / 17870
TSS3_SIGNER_CONTROL_ADDR = 0x777
TSS3_SIGNER_CONTROL_MAGIC = b"\x07\xC7\xC7"
TSS3_SIGNER_BUS = 1

TSS3_NO_LATERAL_REQUEST_ID = 0
TSS3_LTA_LCA_ID = 11
TSS3_LTA_ASSIST_GAIN_RAW = 100
TSS3_INACTIVE_ASSIST_GAIN_RAW = 50
TSS3_LONGITUDINAL_HOST_REQUEST = (0x2D, 0x47)
TSS3_ACCEL_SCALE = 0.001


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


def build_host_application(*, lat_active: bool, target_angle_raw: int,
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

  accel_raw = int(round((accel if long_active else 0.0) / TSS3_ACCEL_SCALE))
  if not -(1 << 15) <= accel_raw < (1 << 15):
    raise ValueError("acceleration must fit signed16")
  set_speed_raw = max(0, min(255, int(round(set_speed_kph)))) if isfinite(set_speed_kph) else 0

  application = bytearray(28)
  application[3] = 0x08
  application[4] = 0x80
  application[6:8] = bytes(TSS3_LONGITUDINAL_HOST_REQUEST)
  accel_bytes = accel_raw.to_bytes(2, "big", signed=True)
  application[8:10] = accel_bytes
  application[10] = set_speed_raw
  application[11:13] = accel_bytes
  application[13:15] = b"\x7F\xFF"
  application[16:18] = b"\x7F\xFF"
  application[18:20] = target_angle_raw.to_bytes(2, "big", signed=True)
  application[20] = 0xC0
  application[21] = TSS3_LTA_LCA_ID if lat_active else TSS3_NO_LATERAL_REQUEST_ID
  application[22] = 0x10
  application[24] = TSS3_LTA_ASSIST_GAIN_RAW if lat_active else TSS3_INACTIVE_ASSIST_GAIN_RAW
  application[26] = request_sequence
  return bytes(application)


# Kept as source-compatible aliases for the exact-F33 analysis tooling.
build_f33_signer_control = build_signer_control
TSS3_F33_SIGNER_CONTROL_ADDR = TSS3_SIGNER_CONTROL_ADDR
TSS3_F33_SIGNER_CONTROL_MAGIC = TSS3_SIGNER_CONTROL_MAGIC
TSS3_F33_SIGNER_BUS = TSS3_SIGNER_BUS
