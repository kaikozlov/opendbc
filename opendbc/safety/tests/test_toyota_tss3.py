#!/usr/bin/env python3
import unittest

from opendbc.can import CANPacker
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.structs import CarParams
from opendbc.car.toyota.carcontroller import get_safety_CP
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.tss3 import build_host_application
from opendbc.car.toyota.values import CAR, EPS_SCALE, CarControllerParams, ToyotaSafetyFlags
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


def fix_toyota_checksum(msg):
  address, data, bus = msg
  payload = bytearray(data)
  payload[-1] = (address + (address >> 8) + len(payload) + sum(payload[:-1])) & 0xFF
  return address, bytes(payload), bus


class TestToyotaTss3CamrySafety(common.CarSafetyTest, common.AngleSteeringSafetyTest,
                                common.LongitudinalAccelSafetyTest):
  TX_MSGS = [[0x777, 1], [0x777, 0], [0x08A, 0], [0x101, 2], [0x412, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x08A, 0x412)}
  FWD_BLACKLISTED_ADDRS = {2: [0x08A, 0x412]}

  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.5
  INACTIVE_ACCEL = 0.0

  STEER_ANGLE_MAX = 1745 * 1024 / 17870
  DEG_TO_CAN = 17870 / 1024
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None
  LATERAL_FREQUENCY = 100

  def setUp(self):
    self.packer = CANPackerSafety("toyota_tss3_pt_generated")
    self.application_packer = CANPacker("toyota_tss3_pt_generated")
    self.safety = libsafety_py.libsafety
    param = EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.TSS3
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self._admin_msg(True)))
    self.angle_cmd_count = 0

    fingerprint = {bus: {} for bus in range(8)}
    self.CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint, [], True, False, False)
    self.VM = VehicleModel(get_safety_CP())
    self.params = CarControllerParams(self.CP)
    self.params.STEER_STEP = 1 / (0.01 * self.LATERAL_FREQUENCY)

  @staticmethod
  def _admin_msg(arm: bool):
    data = bytes((7, 0xC9, 0xA8, int(arm), 0, 0, 0, 0))
    return libsafety_py.make_CANPacket(0x777, 1, data)

  def _tx(self, msg):
    # a rejected CONTROL_REQUEST hands 0x08A back to the FRC, openpilot re-arms like the transport does
    ret = super()._tx(msg)
    if not ret and msg[0].addr == 0x08A:
      super()._tx(self._admin_msg(True))
    return ret

  def _application_msg(self, *, angle: float = 0.0, lat_active: bool = False, accel: float = 0.0):
    angle_raw = round(angle * 17870 / 1024)
    return self._application_raw_msg(angle_raw=angle_raw, lat_active=lat_active, accel=accel)

  def _application_raw_msg(self, *, angle_raw: int = 0, lat_active: bool = False, accel: float = 0.0):
    data = build_host_application(
      self.application_packer,
      lat_active=lat_active,
      target_angle_raw=angle_raw,
      long_active=True,
      accel=accel,
      set_speed_kph=0.0,
      request_sequence=0,
    ) + bytes(4)
    msg = libsafety_py.make_CANPacket(0x08A, 0, data)
    msg[0].fd = 1
    return msg

  def _accel_msg(self, accel: float):
    return self._application_msg(accel=accel)

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    if increment_timer:
      self.safety.set_timer(self.angle_cmd_count * int(1e6 / self.LATERAL_FREQUENCY))
      self.angle_cmd_count += 1
    return self._application_msg(angle=angle, lat_active=enabled)

  def _angle_raw_cmd_msg(self, angle_raw: int):
    self.safety.set_timer(self.angle_cmd_count * int(1e6 / self.LATERAL_FREQUENCY))
    self.angle_cmd_count += 1
    return self._application_raw_msg(angle_raw=angle_raw, lat_active=True)

  def _angle_meas_msg(self, angle: float):
    coarse = round(angle / 1.5)
    fraction = angle - coarse * 1.5
    values = {"STEER_ANGLE": coarse * 1.5, "STEER_FRACTION": fraction}
    return self.packer.make_can_msg_safety("STEER_ANGLE_SENSOR", 0, values)

  def _get_steer_cmd_angle_max(self, speed):
    return min(get_max_angle_vm(max(speed - 1., 1.), self.VM, self.params), 32767 / self.DEG_TO_CAN)

  def test_angle_cmd_when_enabled(self):
    # covered by test_lateral_accel_limit
    pass

  def test_lateral_accel_limit(self):
    for speed in (1., 5., 10., 15., 25., 40.):
      self._reset_speed_measurement(speed + 1.)
      max_angle_raw = min(int(get_max_angle_vm(speed, self.VM, self.params) * self.DEG_TO_CAN) + 1, 1745)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self.safety.set_desired_angle_last(sign * max_angle_raw)
        self.assertTrue(self._tx(self._angle_raw_cmd_msg(sign * max_angle_raw)))

        self.safety.set_controls_allowed(True)
        self.safety.set_desired_angle_last(sign * (max_angle_raw + 1))
        self.assertFalse(self._tx(self._angle_raw_cmd_msg(sign * (max_angle_raw + 1))))

  def test_lateral_jerk_limit(self):
    for speed in (1., 5., 10., 15., 25., 40.):
      self._reset_speed_measurement(speed + 1.)
      max_delta_raw = min(int(get_max_angle_delta_vm(speed, self.VM, self.params) * self.DEG_TO_CAN) + 1, 1745)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self.safety.set_desired_angle_last(0)
        self.assertTrue(self._tx(self._angle_raw_cmd_msg(sign * max_delta_raw)))

        self.safety.set_controls_allowed(True)
        self.safety.set_desired_angle_last(0)
        self.assertFalse(self._tx(self._angle_raw_cmd_msg(sign * (max_delta_raw + 1))))

  def test_angle_rate_budget_tracks_publication_interval(self):
    # too large for one frame, allowed when a signer response was lost and 20ms elapsed
    self._reset_speed_measurement(9.95)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(-549)
    self.safety.set_timer(10_000)
    self.assertTrue(self._tx(self._application_raw_msg(angle_raw=-549, lat_active=True)))
    self.safety.set_timer(20_000)
    self.assertFalse(self._tx(self._application_raw_msg(angle_raw=-520, lat_active=True)))

    self.safety.set_timer(30_000)
    self.assertTrue(self._tx(self._admin_msg(True)))
    self.safety.set_desired_angle_last(-549)
    self.safety.set_timer(40_000)
    self.assertTrue(self._tx(self._application_raw_msg(angle_raw=-549, lat_active=True)))
    self.safety.set_timer(60_000)
    self.assertTrue(self._tx(self._application_raw_msg(angle_raw=-520, lat_active=True)))

  def test_angle_rate_budget_has_nominal_generation_floor(self):
    # signer jitter can send frames less than 10ms apart
    speed = 9.95
    self._reset_speed_measurement(speed + 1.)
    nominal_delta_raw = min(int(get_max_angle_delta_vm(speed, self.VM, self.params) * self.DEG_TO_CAN) + 1, 1745)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    self.safety.set_timer(100_000)
    self.assertTrue(self._tx(self._application_raw_msg(angle_raw=0, lat_active=True)))
    self.safety.set_timer(107_000)
    self.assertTrue(self._tx(self._application_raw_msg(angle_raw=nominal_delta_raw, lat_active=True)))

  def test_vehicle_speed_measurements(self):
    self._common_measurement_test(self._speed_msg, 0, 71.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_private_transport_envelopes(self):
    # the high nibble is the fragment index (8-B), the low nibble is part of the sequence
    for header in (0x80, 0x8F, 0x90, 0x9F, 0xA0, 0xAF, 0xB0, 0xBF):
      msg = libsafety_py.make_CANPacket(0x777, 0, bytes((header, 1, 2, 3, 4, 5, 6, 7)))
      self.assertTrue(self._tx(msg), hex(header))

    invalid_requests = (
      bytes((0xC9, 1, 0, 0, 0, 0, 0, 0)),
      bytes((0xC8, 6 << 5 | 1, 0, 0, 0, 0, 0, 0)),
      bytes((0xC8, 0, 0, 0, 0, 0, 0, 0)),
      bytes((0xC8, 1, 0, 0, 0, 0, 0, 1)),
      bytes((0x7F, 1, 2, 3, 4, 5, 6, 7)),
      bytes((0xC0, 1, 2, 3, 4, 5, 6, 7)),
    )
    for data in invalid_requests:
      self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x777, 0, data)))

    for action in (False, True):
      self.assertTrue(self._tx(self._admin_msg(action)))
    for index in (0, 1, 2, 4, 5, 6, 7):
      data = bytearray(self._admin_msg(True)[0].data)
      data[index] ^= 1
      self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x777, 1, bytes(data))))
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x777, 1, bytes((7, 0xC9, 0xA8, 2, 0, 0, 0, 0)))))

  def test_host_application_schema_and_ownership(self):
    canonical = self._application_raw_msg()
    self.assertTrue(self._tx(canonical))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

    fixed_bytes = (0, 1, 2, 3, 4, 5, 6, 7, 13, 14, 15, 16, 17, 20, 22, 23, 25, 27)
    for index in fixed_bytes:
      data = bytearray(canonical[0].data)[:32]
      data[index] ^= 1
      msg = libsafety_py.make_CANPacket(0x08A, 0, bytes(data))
      msg[0].fd = 1
      self.assertFalse(self.safety.safety_tx_hook(msg), index)
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)
      self.assertTrue(self._tx(self._admin_msg(True)))

    classic = libsafety_py.make_CANPacket(0x08A, 0, bytes(canonical[0].data)[:32])
    self.assertFalse(self.safety.safety_tx_hook(classic))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

  def test_rejected_request_yields_to_frc(self):
    self.assertTrue(self._tx(self._application_raw_msg()))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

    # a rejected frame forwards the FRC's request immediately, and openpilot's stay blocked until re-armed
    self.safety.set_controls_allowed(False)
    self.assertFalse(self.safety.safety_tx_hook(self._application_raw_msg(angle_raw=100, lat_active=True)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)
    self.assertFalse(self.safety.safety_tx_hook(self._application_raw_msg()))

    self.assertTrue(self._tx(self._admin_msg(True)))
    self.assertTrue(self._tx(self._application_raw_msg()))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

  def test_gas_pressed_does_not_block_accel(self):
    # 0x08A must never stop; the brake ECU arbitrates driver gas
    self.safety.set_controls_allowed(True)
    self.safety.set_gas_pressed_prev(True)
    self.assertTrue(self._tx(self._application_msg(accel=self.MAX_ACCEL)))
    self.assertTrue(self._tx(self._application_msg(accel=self.MIN_ACCEL)))
    self.assertFalse(self._tx(self._application_msg(accel=self.MAX_ACCEL + 0.001)))
    self.assertFalse(self._tx(self._application_msg(accel=self.MIN_ACCEL - 0.001)))

  def test_request_plane_watchdog_and_release(self):
    self.assertTrue(self._tx(self._application_raw_msg()))
    self.safety.set_timer(99_999)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)
    self.safety.set_timer(100_001)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

    self.assertTrue(self._tx(self._admin_msg(True)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)
    self.assertTrue(self._tx(self._admin_msg(False)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

  def test_stock_shaped_brake_cancel(self):
    _, cancel_data, _ = fix_toyota_checksum((0x101, bytes((0x88, 0, 0, 0, 0, 0, 0, 0)), 2))
    self.assertTrue(self._tx(libsafety_py.make_CANPacket(0x101, 2, cancel_data)))

    brake_off = bytearray(cancel_data)
    brake_off[0] &= ~0x08
    _, brake_off, _ = fix_toyota_checksum((0x101, bytes(brake_off), 2))
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x101, 2, brake_off)))

    bad_checksum = bytearray(cancel_data)
    bad_checksum[-1] ^= 1
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x101, 2, bytes(bad_checksum))))

  def _user_brake_msg(self, brake):
    return self.packer.make_can_msg_safety("BRAKE_MODULE", 0, {"BRAKE_PRESSED": brake}, fix_toyota_checksum)

  def _speed_msg(self, speed):
    values = {f"WHEEL_SPEED_{wheel}": speed * 3.6 for wheel in ("FR", "FL", "RR", "RL")}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values)

  def _speed_msg_2(self, speed):
    return None

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("GAS_PEDAL", 0, {"GAS_PEDAL_USER": gas})

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety("CONTROL_REQUEST", 2, {"CRUISE_OPERATING_LATCH": enable})


class TestToyotaTss3CamryStockLongitudinalSafety(unittest.TestCase):
  STOCK_08A = bytes.fromhex("0000000880002d47fe462afe467fff007fffff35c000100064003c005db7797f")

  def setUp(self):
    self.application_packer = CANPacker("toyota_tss3_pt_generated")
    self.safety = libsafety_py.libsafety
    param = EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.TSS3 | ToyotaSafetyFlags.STOCK_LONGITUDINAL
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()
    self.safety.set_timer(0)

  @staticmethod
  def _admin_msg(arm: bool):
    return libsafety_py.make_CANPacket(0x777, 1, bytes((7, 0xC9, 0xA8, int(arm), 0, 0, 0, 0)))

  @staticmethod
  def _fd_msg(data: bytes, bus: int):
    msg = libsafety_py.make_CANPacket(0x08A, bus, data)
    msg[0].fd = 1
    return msg

  def _host_msg(self, stock: bytes | None = None, request_sequence: int = 12):
    stock = self.STOCK_08A if stock is None else stock
    application = build_host_application(
      self.application_packer,
      lat_active=False,
      target_angle_raw=0,
      long_active=False,
      accel=0.0,
      set_speed_kph=0.0,
      request_sequence=request_sequence,
      stock_application=stock[:28],
    )
    return self._fd_msg(application + bytes(4), 0)

  def test_stock_longitudinal_requires_recent_frc_application(self):
    self.assertFalse(self.safety.safety_tx_hook(self._admin_msg(True)))
    self.assertTrue(self.safety.safety_rx_hook(self._fd_msg(self.STOCK_08A, 2)))
    self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(True)))
    self.assertTrue(self.safety.safety_tx_hook(self._host_msg()))

    self.safety.set_timer(100_001)
    self.assertFalse(self.safety.safety_tx_hook(self._host_msg()))
    self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(False)))
    self.assertFalse(self.safety.safety_tx_hook(self._admin_msg(True)))

  def test_stock_longitudinal_fields_must_match_frc(self):
    self.assertTrue(self.safety.safety_rx_hook(self._fd_msg(self.STOCK_08A, 2)))
    self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(True)))

    for index in (3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 17, 20, 22, 23, 27):
      with self.subTest(index=index):
        msg = self._host_msg()
        msg[0].data[index] ^= 1
        self.assertFalse(self.safety.safety_tx_hook(msg))
        self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)
        self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(True)))

    self.assertTrue(self.safety.safety_tx_hook(self._host_msg()))

  def test_one_frc_application_allows_multiple_100hz_host_generations(self):
    self.assertTrue(self.safety.safety_rx_hook(self._fd_msg(self.STOCK_08A, 2)))
    self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(True)))
    for request_sequence in range(3):
      self.safety.set_timer(request_sequence * 10_000)
      self.assertTrue(self.safety.safety_tx_hook(self._host_msg(request_sequence=request_sequence)))


if __name__ == "__main__":
  unittest.main()
