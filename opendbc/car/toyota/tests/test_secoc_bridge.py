import unittest

from opendbc.car.secoc import add_mac28_zero_marker


class TestEphemeralSecocBridgeMarker(unittest.TestCase):
  def test_marker_preserves_authentic_payload_and_freshness_nibble(self):
    msg = (0x2E4, bytes.fromhex("12345678deadbeef"), 0)
    result = add_mac28_zero_marker(reset_cnt=0x1235, msg_cnt=0xA6, msg=msg)
    self.assertEqual(result[0], 0x2E4)
    self.assertEqual(result[2], 0)
    self.assertEqual(result[1][:4], bytes.fromhex("12345678"))
    self.assertEqual(result[1][4], ((((0xA6 & 3) << 2) | (0x1235 & 3)) << 4))
    self.assertEqual(result[1][4] & 0x0F, 0)
    self.assertEqual(result[1][5:], bytes(3))

  def test_marker_rejects_short_authentic_payload(self):
    with self.assertRaises(ValueError):
      add_mac28_zero_marker(reset_cnt=0, msg_cnt=0, msg=(0x2E4, b"\x00\x01\x02", 0))


if __name__ == "__main__":
  unittest.main()
