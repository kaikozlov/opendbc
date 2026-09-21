from opendbc.car.secoc import add_mac, attach_authenticator, build_authentication_data, build_freshness_value, build_sync_mac


KEY = bytes.fromhex("00112233445566778899aabbccddeeff")


def test_secoc_primitives_support_arbitrary_application_lengths():
  application = bytes.fromhex("2000000000000000000000000000000000000000000000000000000c")
  domain = build_authentication_data(0x08A, application, 620, 1109, 8)

  assert build_freshness_value(620, 1109, 8) == bytes.fromhex("026c00455084")
  assert domain == bytes.fromhex("008a") + application + bytes.fromhex("026c00455084")
  assert attach_authenticator(application, 1109, 8, bytes.fromhex("d64e2a5e")) == application + bytes.fromhex("1d64e2a5")


def test_add_mac_preserves_known_key_behavior():
  msg = (0x131, bytes.fromhex("1234567800000000"), 0)

  assert add_mac(KEY, 620, 1109, 8, msg) == (0x131, bytes.fromhex("1234567818f5841d"), 0)
  assert build_sync_mac(KEY, 620, 1109) == 0x00F74EA6


def test_attach_authenticator_rejects_short_cmac():
  try:
    attach_authenticator(b"application", 0, 0, b"\x00\x01\x02")
  except ValueError as exc:
    assert str(exc) == "SecOC CMAC must contain at least 32 bits"
  else:
    raise AssertionError("short CMAC accepted")
