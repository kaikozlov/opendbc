from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.structs import CarParams

SteerControlType = CarParams.SteerControlType


def create_steer_command(packer, steer, steer_req):
  """Creates a CAN message for the Toyota Steer Command."""

  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "SET_ME_1": 1,
  }
  return packer.make_can_msg("STEERING_LKA", 0, values)


def create_lta_steer_command(packer, steer_control_type, steer_angle, steer_req, frame, torque_wind_down):
  """Creates a CAN message for the Toyota LTA Steer Command."""

  values = {
    "COUNTER": frame + 128,
    "SETME_X1": 1,  # suspected LTA feature availability
    # 1 for TSS 2.5 cars, 3 for TSS 2.0. Send based on whether we're using LTA for lateral control
    "SETME_X3": 1 if steer_control_type == SteerControlType.angle else 3,
    "PERCENTAGE": 100,
    "TORQUE_WIND_DOWN": torque_wind_down,
    "ANGLE": 0,
    "STEER_ANGLE_CMD": steer_angle,
    "STEER_REQUEST": steer_req,
    "STEER_REQUEST_2": steer_req,
    "CLEAR_HOLD_STEERING_ALERT": 0,
  }
  return packer.make_can_msg("STEERING_LTA", 0, values)


def create_lta_steer_command_2(packer, frame):
  values = {
    "COUNTER": frame + 128,
  }
  return packer.make_can_msg("STEERING_LTA_2", 0, values)


def create_accel_command(packer, accel, pcm_cancel, permit_braking, standstill_req, lead, acc_type, fcw_alert, distance):
  # TODO: find the exact canceling bit that does not create a chime
  values = {
    "ACCEL_CMD": accel,
    "ACC_TYPE": acc_type,
    "DISTANCE": distance,
    "MINI_CAR": lead,
    "PERMIT_BRAKING": permit_braking,
    "RELEASE_STANDSTILL": not standstill_req,
    "CANCEL_REQ": pcm_cancel,
    "ALLOW_LONG_PRESS": 1,
    "ACC_CUT_IN": fcw_alert,  # only shown when ACC enabled
  }
  return packer.make_can_msg("ACC_CONTROL", 0, values)


def create_accel_command_2(packer, accel):
  values = {
    "ACCEL_CMD": accel,
  }
  return packer.make_can_msg("ACC_CONTROL_2", 0, values)


def create_tss3_accel_command(template: dict[str, float], accel: float | None, *, camry_b12: bool = False):
  """Relay one live FRC 0x160 image, optionally replacing its platform request fields."""
  data = bytearray(32)
  data[2] = int(template["COUNTER"])
  for i in range(3, 32):
    data[i] = int(template[f"BYTE_{i}"])

  if accel is not None:
    raw = max(-16384, min(16383, round(accel / 0.001))) & 0x7FFF
    data[4] = (data[4] & 0x80) | (raw >> 8)
    data[5] = raw & 0xFF
    if camry_b12:
      # Camry additionally uses an inverted signed-7 request at 0.1 m/s^2/count.
      coarse = max(-64, min(63, round(-accel / 0.1)))
      data[12] = (data[12] & 0x80) | (coarse & 0x7F)
  # Standard Profile 5. At this fixed length this is wire-equivalent to the
  # old init=0/DataID=0x444A expression, but also generalizes to the radar PDUs.
  data[0:2] = toyota_e2e_p05_checksum(0x160, data).to_bytes(2, "little")
  return 0x160, bytes(data), 0


def create_tss3_brake_cancel_command(packer, stock_brake):
  """Clone live 0x101 state and assert only the source-real brake-cancel bit."""
  values = {
    "SET_ME_1": stock_brake["SET_ME_1"],
    "BRAKE_PRESSED": 1,
    "BRAKE_BYTE_1": stock_brake["BRAKE_BYTE_1"],
    "BRAKE_BYTE_3": stock_brake["BRAKE_BYTE_3"],
  }
  return packer.make_can_msg("BRAKE_MODULE", 2, values)


def create_tss3_hud_command(stock_hud, left_line: bool, right_line: bool, lat_active: bool, steer_alert: bool):
  """Clone the live FRC HUD frame and render only the recovered openpilot HUD subset."""
  data = bytearray(int(stock_hud[f"BYTE_{i}"]) for i in range(8))

  # The ordinary road-state 0x412 alphabet is recovered on the maintainer
  # Camry: inactive recognized/missing lanes are nibble 1/2, active recognized
  # lanes are nibble 4, with B0 low mode 2->4 and B4 2->1 under lateral control.
  # Preserve startup/noncanonical frames rather than assigning unknown states.
  if data[0] not in (0x12, 0x14) or data[4] not in (1, 2):
    return 0x412, bytes(data), 0

  visible_line = 4 if lat_active else 1
  if left_line == right_line:
    # Symmetric visibility is orientation-free.
    high_state = low_state = visible_line if left_line else 2
  else:
    # Existing road data does not yet prove which B3 nibble is left versus
    # right. Preserve the stock per-side orientation for asymmetric requests.
    def normalize_stock_lane(state: int) -> int:
      return visible_line if state in (1, 4) else state

    high_state = normalize_stock_lane(data[3] >> 4)
    low_state = normalize_stock_lane(data[3] & 0x0F)

  data[0] = (data[0] & ~0x06) | (0x04 if lat_active else 0x02)
  data[3] = (high_state << 4) | low_state
  data[4] = 1 if lat_active else 2

  # B1[3:2] is the source-real hands-off visual warning. Replace it with
  # openpilot DM's steer-required visual. B2[6] is a later Toyota escalation
  # stage; no TSS3 audible/chime contract is recovered, so keep it suppressed.
  data[1] = (data[1] & ~0x0C) | (0x0C if steer_alert else 0)
  data[2] &= ~0x40
  return 0x412, bytes(data), 0


def create_pcs_commands(packer, accel, active, mass):
  values1 = {
    "COUNTER": 0,
    "FORCE": round(min(accel, 0) * mass * 2),
    "STATE": 3 if active else 0,
    "BRAKE_STATUS": 0,
    "PRECOLLISION_ACTIVE": 1 if active else 0,
  }
  msg1 = packer.make_can_msg("PRE_COLLISION", 0, values1)

  values2 = {
    "DSS1GDRV": min(accel, 0),     # accel
    "PCSALM": 1 if active else 0,  # goes high same time as PRECOLLISION_ACTIVE
    "IBTRGR": 1 if active else 0,  # unknown
    "PBATRGR": 1 if active else 0, # noisy actuation bit?
    "PREFILL": 1 if active else 0, # goes on and off before DSS1GDRV
    "AVSTRGR": 1 if active else 0,
  }
  msg2 = packer.make_can_msg("PRE_COLLISION_2", 0, values2)

  return [msg1, msg2]


def create_acc_cancel_command(packer):
  values = {
    "GAS_RELEASED": 0,
    "CRUISE_ACTIVE": 0,
    "ACC_BRAKING": 0,
    "ACCEL_NET": 0,
    "CRUISE_STATE": 0,
    "CANCEL_REQ": 1,
  }
  return packer.make_can_msg("PCM_CRUISE", 0, values)


def create_fcw_command(packer, fcw):
  values = {
    "PCS_INDICATOR": 1,  # PCS turned off
    "FCW": fcw,
    "SET_ME_X20": 0x20,
    "SET_ME_X10": 0x10,
    "PCS_OFF": 1,
    "PCS_SENSITIVITY": 0,
  }
  return packer.make_can_msg("PCS_HUD", 0, values)


def create_ui_command(packer, steer, chime, left_line, right_line, left_lane_depart, right_lane_depart, enabled, stock_lkas_hud):
  values = {
    "TWO_BEEPS": chime,
    "LDA_ALERT": steer,
    "RIGHT_LINE": 3 if right_lane_depart else 1 if right_line else 2,
    "LEFT_LINE": 3 if left_lane_depart else 1 if left_line else 2,
    "BARRIERS": 1 if enabled else 0,

    # static signals
    "SET_ME_X02": 2,
    "SET_ME_X01": 1,
    "LKAS_STATUS": 1,
    "REPEATED_BEEPS": 0,
    "LANE_SWAY_FLD": 7,
    "LANE_SWAY_BUZZER": 0,
    "LANE_SWAY_WARNING": 0,
    "LDA_FRONT_CAMERA_BLOCKED": 0,
    "TAKE_CONTROL": 0,
    "LANE_SWAY_SENSITIVITY": 2,
    "LANE_SWAY_TOGGLE": 1,
    "LDA_ON_MESSAGE": 0,
    "LDA_MESSAGES": 0,
    "LDA_SA_TOGGLE": 1,
    "LDA_SENSITIVITY": 2,
    "LDA_UNAVAILABLE": 0,
    "LDA_MALFUNCTION": 0,
    "LDA_UNAVAILABLE_QUIET": 0,
    "ADJUSTING_CAMERA": 0,
    "LDW_EXIST": 1,
  }

  # lane sway functionality
  # not all cars have LKAS_HUD — update with camera values if available
  if len(stock_lkas_hud):
    values.update({s: stock_lkas_hud[s] for s in [
      "LANE_SWAY_FLD",
      "LANE_SWAY_BUZZER",
      "LANE_SWAY_WARNING",
      "LANE_SWAY_SENSITIVITY",
      "LANE_SWAY_TOGGLE",
    ]})

  return packer.make_can_msg("LKAS_HUD", 0, values)


def toyota_checksum(address: int, sig, d: bytearray) -> int:
  s = len(d)
  addr = address
  while addr:
    s += addr & 0xFF
    addr >>= 8
  for i in range(len(d) - 1):
    s += d[i]
  return s & 0xFF


def toyota_e2e_p05_checksum(address: int, data: bytes | bytearray) -> int:
  """Toyota's native E2E P05: init FFFF, CRC LE, implicit DataID=CAN ID."""
  crc = 0xFFFF
  for byte in (*data[2:], address & 0xFF, (address >> 8) & 0xFF):
    crc = ((crc << 8) ^ CRC16_XMODEM[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
  return crc


def toyota_tss3_checksum(address: int, sig, data: bytearray) -> int:
  # The TSS3 DBC contains both 16-bit E2E and inherited 8-bit additive fields.
  return toyota_e2e_p05_checksum(address, data) if sig.size == 16 else toyota_checksum(address, sig, data)
