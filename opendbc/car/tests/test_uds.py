import unittest

from opendbc.car.uds import CanClient, IsoTpMessage


def make_can_client(batches, sub_addr):
  return CanClient(lambda *_: None, lambda: batches.pop(0) if batches else [], 0x750, 0x758, 0, sub_addr=sub_addr)


class TestSubAddressFiltering(unittest.TestCase):
  def test_recv_ignores_frames_for_other_sub_addresses(self):
    # late/stray responses from another ECU on a shared address must be dropped, not fatal
    frames = [
      (0x758, bytes([0x2A, 0x03, 0x54]), 0),  # stray frame addressed to sub-address 0x2A
      (0x758, bytes([0xD3, 0x03, 0x54]), 0),  # our response, sub-address 0xD3
    ]
    client = CanClient(lambda *_: None, lambda: frames, 0x750, 0x758, 0, sub_addr=0xD3)
    self.assertEqual(list(client.recv()), [bytes([0x03, 0x54])])

  def test_isotp_transaction_survives_stray_sub_address_frame(self):
    # clear request to sub-address 0xD3 on Toyota's shared 0x750 bus; the first can_recv
    # batch (the drain in IsoTpMessage.send) is empty, then a late 0x2A frame and our
    # single-frame 0x54 response arrive together
    batches = [
      [],
      [
        (0x758, bytes([0x2A, 0x02, 0x44, 0x00]), 0),
        (0x758, bytes([0xD3, 0x02, 0x54, 0x00]), 0),
      ],
    ]
    msg = IsoTpMessage(make_can_client(batches, 0xD3), timeout=0.1)
    msg.send(bytes([0x14, 0xFF, 0xFF, 0xFF]))
    resp, _ = msg.recv()
    self.assertEqual(resp, b"\x54\x00")


if __name__ == "__main__":
  unittest.main()
