import struct

from Crypto.Hash import CMAC
from Crypto.Cipher import AES


def add_mac(key, trip_cnt, reset_cnt, msg_cnt, msg):
  # TODO: clean up conversion to and from hex

  addr, payload, bus = msg
  reset_flag = reset_cnt & 0b11
  msg_cnt_flag = msg_cnt & 0b11
  payload = payload[:4]

  # Step 1: Build Freshness Value (48 bits)
  # [Trip Counter (16 bit)][[Reset Counter (20 bit)][Message Counter (8 bit)][Reset Flag (2 bit)][Padding (2 bit)]
  freshness_value = struct.pack('>HI', trip_cnt, (reset_cnt << 12) | ((msg_cnt & 0xff) << 4) | (reset_flag << 2))

  # Step 2: Build data to authenticate (96 bits)
  # [Message ID (16 bits)][Payload (32 bits)][Freshness Value (48 bits)]
  to_auth = struct.pack('>H', addr) + payload + freshness_value

  # Step 3: Calculate CMAC (28 bit)
  cmac = CMAC.new(key, ciphermod=AES)
  cmac.update(to_auth)
  mac = cmac.digest().hex()[:7] # truncated MAC

  # Step 4: Build message
  # [Payload (32 bit)][Message Counter Flag (2 bit)][Reset Flag (2 bit)][Authenticator (28 bit)]
  msg_cnt_rst_flag = struct.pack('>B', (msg_cnt_flag << 2) | reset_flag).hex()[1]
  msg = payload.hex() + msg_cnt_rst_flag + mac
  payload = bytes.fromhex(msg)

  return (addr, payload, bus)


def add_mac28_zero_marker(reset_cnt, msg_cnt, msg):
  """Build a classic Toyota SecOC envelope with an all-zero MAC28 marker.

  The high nibble of byte 4 remains the stock message-counter/reset-flag nibble;
  only the 28 authenticator bits are zero. This is consumed only by an explicitly
  installed EPS-side ephemeral bridge.
  """
  addr, payload, bus = msg
  if len(payload) < 4:
    raise ValueError("Toyota SecOC marker requires at least the four-byte authentic payload")
  reset_flag = reset_cnt & 0b11
  msg_cnt_flag = msg_cnt & 0b11
  flags = ((msg_cnt_flag << 2) | reset_flag) << 4
  return (addr, payload[:4] + bytes((flags, 0, 0, 0)), bus)


def build_sync_mac(key, trip_cnt, reset_cnt, id_=0xf):
  id_ = struct.pack('>H', id_) # 16
  trip_cnt = struct.pack('>H', trip_cnt) # 16
  reset_cnt = struct.pack('>I', reset_cnt << 12)[:-1] # 20 + 4 padding

  to_auth = id_ + trip_cnt + reset_cnt # SecOC 11.4.1.1 page 138

  cmac = CMAC.new(key, ciphermod=AES)
  cmac.update(to_auth)

  msg = "0" + cmac.digest().hex()[:7]
  msg = bytes.fromhex(msg)
  return struct.unpack('>I', msg)[0]
