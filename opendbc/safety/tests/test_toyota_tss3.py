#!/usr/bin/env python3
import unittest

from opendbc.can import CANPacker
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.structs import CarParams
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
  LATERAL_FREQUENCY = 40

  def setUp(self):
    self.packer = CANPackerSafety("toyota_tss3_pt_generated")
    self.application_packer = CANPacker("toyota_tss3_pt_generated")
    self.safety = libsafety_py.libsafety
    param = (EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] |
             ToyotaSafetyFlags.F33 | ToyotaSafetyFlags.TSS3_08A_HOST)
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self._admin_msg(True)))
    self.angle_cmd_count = 0

    fingerprint = {bus: {} for bus in range(8)}
    self.CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint, [], True, False, False)
    self.VM = VehicleModel(self.CP)
    self.params = CarControllerParams(self.CP)
    self.params.STEER_STEP = 1 / (0.01 * self.LATERAL_FREQUENCY)

  @staticmethod
  def _admin_msg(arm: bool):
    data = bytes((7, 0xC9, 0xA8, int(arm), 0, 0, 0, 0))
    return libsafety_py.make_CANPacket(0x777, 1, data)

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
    # Vehicle-model angle limits are speed-dependent and are checked below.
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

  def test_vehicle_speed_measurements(self):
    self._common_measurement_test(self._speed_msg, 0, 71.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_private_transport_envelopes(self):
    for fragment in range(6):
      header = (fragment << 5) | 1
      msg = libsafety_py.make_CANPacket(0x777, 0, bytes((0xC8, header, 0, 0, 0, 0, 0, 0)))
      self.assertTrue(self._tx(msg))

    invalid_oracle = (
      bytes((0xC9, 1, 0, 0, 0, 0, 0, 0)),
      bytes((0xC8, 6 << 5 | 1, 0, 0, 0, 0, 0, 0)),
      bytes((0xC8, 0, 0, 0, 0, 0, 0, 0)),
      bytes((0xC8, 1, 0, 0, 0, 0, 0, 1)),
    )
    for data in invalid_oracle:
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
      data = bytearray(canonical[0].data)
      data[index] ^= 1
      msg = libsafety_py.make_CANPacket(0x08A, 0, bytes(data))
      msg[0].fd = 1
      self.assertFalse(self._tx(msg), index)
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

    classic = libsafety_py.make_CANPacket(0x08A, 0, bytes(canonical[0].data))
    self.assertFalse(self._tx(classic))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

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
    return self.packer.make_can_msg_safety("TSS3_CONTROL_REQUEST", 2, {"CRUISE_OPERATING_LATCH": enable})


class TestToyotaTss3CorollaSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest,
                                  common.LongitudinalAccelSafetyTest):
  TX_MSGS = [[0x777, 1], [0x101, 1]]
  RELAY_MALFUNCTION_ADDRS = {}

  LONGITUDINAL = False
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.5
  INACTIVE_ACCEL = 0.0

  STEER_ANGLE_MAX = 1745 * 1024 / 17870
  STEER_ANGLE_TEST_MAX = STEER_ANGLE_MAX - 5
  DEG_TO_CAN = 17870 / 1024
  ANGLE_RATE_BP = [5., 25., 25.]
  ANGLE_RATE_UP = [0.15, 0.075, 0.075]
  ANGLE_RATE_DOWN = [0.18, 0.13, 0.13]

  def setUp(self):
    self.packer = CANPackerSafety("toyota_tss3_pt_generated")
    self.safety = libsafety_py.libsafety
    param = (EPS_SCALE[CAR.TOYOTA_COROLLA_TSS3] | ToyotaSafetyFlags.TSS3_SIGNER |
             ToyotaSafetyFlags.COROLLA_HF | ToyotaSafetyFlags.STOCK_LONGITUDINAL)
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    angle_raw = round(angle * self.DEG_TO_CAN)
    sequence = 1 if enabled else 0
    data = b"\x07\xC7\xC7" + bytes((sequence,)) + angle_raw.to_bytes(2, "big", signed=True) + bytes(2)
    return libsafety_py.make_CANPacket(0x777, 1, data)

  def _angle_meas_msg(self, angle: float):
    coarse = round(angle / 1.5)
    fraction = angle - coarse * 1.5
    values = {"STEER_ANGLE": coarse * 1.5, "STEER_FRACTION": fraction}
    return self.packer.make_can_msg_safety("STEER_ANGLE_SENSOR", 1, values)

  def _accel_msg(self, accel: float):
    values = {
      "LONGITUDINAL_REQUEST_ID_A": 11,
      "LONGITUDINAL_ALLOCATION_METHOD_A": 1,
      "LONGITUDINAL_REQUEST_ID_B": 17,
      "LONGITUDINAL_ALLOCATION_METHOD_B": 3,
      "LONGITUDINAL_REQUEST_ACCEL_A": accel,
      "LONGITUDINAL_REQUEST_ACCEL_B": accel,
    }
    return self.packer.make_can_msg_safety("TSS3_CONTROL_REQUEST", 1, values)

  def _user_brake_msg(self, brake):
    return self.packer.make_can_msg_safety("BRAKE_MODULE", 1, {"BRAKE_PRESSED": brake}, fix_toyota_checksum)

  def _speed_msg(self, speed):
    values = {f"WHEEL_SPEED_{wheel}": speed * 3.6 for wheel in ("FR", "FL", "RR", "RL")}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 1, values)

  def _speed_msg_2(self, speed):
    return None

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("GAS_PEDAL", 1, {"GAS_PEDAL_USER": gas})

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety("TSS3_CONTROL_REQUEST", 1, {"COROLLA_ACC_ENGAGED": enable})

  def test_vehicle_speed_measurements(self):
    self._common_measurement_test(self._speed_msg, 0, 71.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_signer_control_and_brake_cancel_envelopes(self):
    self.safety.set_controls_allowed(True)
    c7 = self._angle_cmd_msg(0, True)
    self.assertTrue(self._tx(c7))
    for index in (0, 1, 2, 6, 7):
      data = bytearray(c7[0].data)
      data[index] ^= 1
      self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x777, 1, bytes(data))))

    _, cancel_data, _ = fix_toyota_checksum((0x101, bytes((0x88, 0, 0, 0, 0, 0, 0, 0)), 1))
    self.assertTrue(self._tx(libsafety_py.make_CANPacket(0x101, 1, cancel_data)))
    brake_off = bytearray(cancel_data)
    brake_off[0] &= ~0x08
    _, brake_off, _ = fix_toyota_checksum((0x101, bytes(brake_off), 1))
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x101, 1, brake_off)))


if __name__ == "__main__":
  unittest.main()
