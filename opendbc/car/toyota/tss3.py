"""TSS3 EPS-resident signer sideband helpers."""

TSS3_B6_TARGET_ANGLE_SCALE_DEG = 1024 / 17870
TSS3_SIGNER_CONTROL_ADDR = 0x777
TSS3_SIGNER_CONTROL_MAGIC = b"\x07\xC7\xC7"
TSS3_SIGNER_BUS = 1

TSS3_LATERAL_SOURCE_IDS = (0, 4, 11, 18)  # No Request, LDA, LTA/LCA, SDG
TSS3_NO_LATERAL_REQUEST_ID = 0
TSS3_LTA_LCA_ID = 11
TSS3_LTA_ASSIST_GAIN_RAW = 100
TSS3_REPLACEABLE_LONGITUDINAL_REQUESTS = ((0x00, 0x12), (0x2C, 0x46), (0x2D, 0x47), (0x2D, 0x67))
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


def build_request_application(native_application: bytes, *, lat_active: bool, target_angle_raw: int,
                              long_control: bool, accel: float) -> bytes:
  """Merge openpilot actuation into one source-real TSS3 0x08A application.

  The native generation retains unowned lifecycle, set-speed and arbitration
  metadata. While openpilot is engaged, known ordinary lateral requests are
  encoded as ID11/gain100 when active and ID0 at the measured angle when
  inactive. The inactive request preserves Toyota's source gain companion.
  Engaged openpilot longitudinal promotes the known no-request, ordinary-DRCC,
  driver-override allocation and delayed-hold states to the normal ID11/ID17
  request and owns both acceleration bounds. ``accel`` is zero while
  longitudinal is inactive, matching openpilot's normal driver-override
  contract. Other Toyota request tuples remain unchanged.
  """
  if len(native_application) != 28:
    raise ValueError("native 0x08A application must be 28 bytes")

  application = bytearray(native_application)
  native_id = application[21] & 0x3F
  if native_id in TSS3_LATERAL_SOURCE_IDS:
    application[18:20] = target_angle_raw.to_bytes(2, "big", signed=True)
    application[21] = (application[21] & 0xC0) | (TSS3_LTA_LCA_ID if lat_active else TSS3_NO_LATERAL_REQUEST_ID)
    if lat_active:
      application[24] = TSS3_LTA_ASSIST_GAIN_RAW
  elif lat_active:
    # Unknown request identities can represent Toyota interventions. Never
    # overwrite one with ordinary openpilot lateral control.
    raise ValueError(f"unsupported native lateral request ID {native_id}")

  if long_control and (application[6], application[7]) in TSS3_REPLACEABLE_LONGITUDINAL_REQUESTS:
    application[4] &= ~0x20  # leave Toyota's delayed-hold substate
    application[6:8] = bytes(TSS3_LONGITUDINAL_HOST_REQUEST)
    accel_raw = int(round(accel / TSS3_ACCEL_SCALE))
    if not -(1 << 15) <= accel_raw < (1 << 15):
      raise ValueError("acceleration must fit signed16")
    accel_bytes = accel_raw.to_bytes(2, "big", signed=True)
    application[8:10] = accel_bytes
    application[11:13] = accel_bytes

  return bytes(application)


# Kept as source-compatible aliases for the exact-F33 analysis tooling.
build_f33_signer_control = build_signer_control
TSS3_F33_SIGNER_CONTROL_ADDR = TSS3_SIGNER_CONTROL_ADDR
TSS3_F33_SIGNER_CONTROL_MAGIC = TSS3_SIGNER_CONTROL_MAGIC
TSS3_F33_SIGNER_BUS = TSS3_SIGNER_BUS
