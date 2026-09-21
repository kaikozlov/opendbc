import struct

from Crypto.Hash import CMAC
from Crypto.Cipher import AES


def build_freshness_value(trip_cnt: int, reset_cnt: int, msg_cnt: int) -> bytes:
  """Build Toyota's 48-bit SecOC freshness value."""
  reset_flag = reset_cnt & 0b11
  return struct.pack('>HI', trip_cnt, (reset_cnt << 12) | ((msg_cnt & 0xff) << 4) | (reset_flag << 2))


def build_authentication_data(addr: int, payload: bytes, trip_cnt: int, reset_cnt: int, msg_cnt: int) -> bytes:
  """Build the complete address + application + freshness CMAC domain."""
  return struct.pack('>H', addr) + payload + build_freshness_value(trip_cnt, reset_cnt, msg_cnt)


def attach_authenticator(payload: bytes, reset_cnt: int, msg_cnt: int, cmac: bytes) -> bytes:
  """Append freshness flags and the high 28 bits of a SecOC CMAC."""
  if len(cmac) < 4:
    raise ValueError("SecOC CMAC must contain at least 32 bits")

  freshness_flags = ((msg_cnt & 0b11) << 2) | (reset_cnt & 0b11)
  authenticator = int.from_bytes(cmac[:4], 'big') >> 4
  return payload + ((freshness_flags << 28) | authenticator).to_bytes(4, 'big')


def add_mac(key, trip_cnt, reset_cnt, msg_cnt, msg):
  addr, payload, bus = msg
  payload = payload[:4]

  cmac = CMAC.new(key, ciphermod=AES)
  cmac.update(build_authentication_data(addr, payload, trip_cnt, reset_cnt, msg_cnt))
  payload = attach_authenticator(payload, reset_cnt, msg_cnt, cmac.digest())

  return (addr, payload, bus)


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
