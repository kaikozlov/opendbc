"""Exact-F33 EPS signer sideband helpers."""

TSS3_B6_TARGET_ANGLE_SCALE_DEG = 1024 / 17870
TSS3_F33_SIGNER_CONTROL_ADDR = 0x1FDC0002
TSS3_F33_SIGNER_CONTROL_MAGIC = b"\x00\xC7"
TSS3_F33_SIGNER_BUS = 1


def target_angle_deg_to_raw(angle_deg: float) -> int:
  return int(round(angle_deg / TSS3_B6_TARGET_ANGLE_SCALE_DEG))


def build_f33_signer_control(target_angle_raw: int, control_sequence: int) -> tuple[int, bytes, int]:
  """Build the bounded sideband consumed by the F33 EPS-resident signer.

  Zero is the neutral sequence. Active commands use 1..255 and wrap without
  passing through zero.
  """
  if not -(1 << 15) <= target_angle_raw < (1 << 15):
    raise ValueError("target angle must fit signed16")
  if not 0 <= control_sequence <= 0xFF:
    raise ValueError("signer control sequence must be 0..255")

  data = (TSS3_F33_SIGNER_CONTROL_MAGIC + bytes((control_sequence, 0)) +
          target_angle_raw.to_bytes(2, "big", signed=True) + bytes(2))
  return TSS3_F33_SIGNER_CONTROL_ADDR, data, TSS3_F33_SIGNER_BUS
