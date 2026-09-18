#pragma once

#include "opendbc/safety/declarations.h"

// Stock longitudinal
#define TOYOTA_BASE_TX_MSGS \
  {0x191, 0, 8, .check_relay = true}, {0x412, 0, 8, .check_relay = true}, {0x1D2, 0, 8, .check_relay = false},  /* LKAS + LTA + PCM cancel cmd */  \

#define TOYOTA_COMMON_TX_MSGS \
  TOYOTA_BASE_TX_MSGS \
  {0x2E4, 0, 5, .check_relay = true}, \
  {0x343, 0, 8, .check_relay = false},  /* ACC cancel cmd */  \

#define TOYOTA_COMMON_SECOC_TX_MSGS \
  TOYOTA_BASE_TX_MSGS \
  {0x2E4, 0, 8, .check_relay = true}, {0x131, 0, 8, .check_relay = true}, \
  {0x343, 0, 8, .check_relay = false},  /* ACC cancel cmd */ \

#define TOYOTA_COMMON_LONG_TX_MSGS \
  TOYOTA_COMMON_TX_MSGS \
  /* DSU bus 0 */ \
  {0x283, 0, 7, .check_relay = false}, {0x2E6, 0, 8, .check_relay = false}, {0x2E7, 0, 8, .check_relay = false}, {0x33E, 0, 7, .check_relay = false}, \
  {0x344, 0, 8, .check_relay = false}, {0x365, 0, 7, .check_relay = false}, {0x366, 0, 7, .check_relay = false}, {0x4CB, 0, 8, .check_relay = false}, \
  /* DSU bus 1 */ \
  {0x128, 1, 6, .check_relay = false}, {0x141, 1, 4, .check_relay = false}, {0x160, 1, 8, .check_relay = false}, {0x161, 1, 7, .check_relay = false}, \
  {0x470, 1, 4, .check_relay = false}, \
  /* PCS_HUD */                        \
  {0x411, 0, 8, .check_relay = false}, \
  /* radar diagnostic address */       \
  {0x750, 0, 8, .check_relay = false}, \
  /* ACC */                            \
  {0x343, 0, 8, .check_relay = true},  \

#define TOYOTA_COMMON_SECOC_LONG_TX_MSGS \
  TOYOTA_COMMON_SECOC_TX_MSGS \
  {0x343, 0, 8, .check_relay = true}, \
  {0x183, 0, 8, .check_relay = true},  /* ACC_CONTROL_2 */ \

#define TOYOTA_COMMON_RX_CHECKS(lta)                                                                                                       \
  {.msg = {{ 0xaa, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},      \
  {.msg = {{0x260, 0, 8, 50U, .ignore_counter = true, .ignore_quality_flag=!(lta)}, { 0 }, { 0 }}},  \

#define TOYOTA_RX_CHECKS(lta)                                                                                                               \
  TOYOTA_COMMON_RX_CHECKS(lta)                                                                                                              \
  {.msg = {{0x1D2, 0, 8, 33U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            \
  {.msg = {{0x226, 0, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  { 0 }, { 0 }}},  \

#define TOYOTA_ALT_BRAKE_RX_CHECKS(lta)                                                                                                    \
  TOYOTA_COMMON_RX_CHECKS(lta)                                                                                                             \
  {.msg = {{0x1D2, 0, 8, 33U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                           \
  {.msg = {{0x224, 0, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define TOYOTA_SECOC_RX_CHECKS                                                                                                             \
  TOYOTA_COMMON_RX_CHECKS(false)                                                                                                           \
  {.msg = {{0x176, 0, 8, 32U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                           \
  {.msg = {{0x116, 0, 8, 42U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0x101, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

static bool toyota_secoc = false;
static bool toyota_alt_brake = false;
static bool toyota_stock_longitudinal = false;
static bool toyota_lta = false;
static bool toyota_tss3_signer = false;
static bool toyota_corolla_hf = false;
static bool toyota_tss3_08a_host = false;
static bool toyota_tss3_08a_signed = false;
static bool toyota_tss3_08a_replacement_active = false;
static bool toyota_tss3_08a_native_valid = false;
static uint8_t toyota_tss3_08a_native_app[28] = {0};
static uint8_t toyota_tss3_08a_native_app_prev1[28] = {0};
static uint8_t toyota_tss3_08a_native_app_prev2[28] = {0};
static uint8_t toyota_tss3_08a_native_history = 0U;
static uint8_t toyota_tss3_08a_next_b26 = 0U;
static uint32_t toyota_tss3_08a_last_tx_ts = 0U;
static bool toyota_tss3_08a_sync_valid = false;
static uint32_t toyota_tss3_08a_reset_counter = 0U;
static uint32_t toyota_tss3_08a_active_reset_counter = 0U;
static bool toyota_tss3_08a_msg_low2_valid = false;
static uint8_t toyota_tss3_08a_msg_low2 = 0U;
static uint8_t toyota_tss3_08a_oracle_next_cf = 0U;

const uint32_t TOYOTA_TSS3_08A_REPLACEMENT_TIMEOUT_US = 40000U;
static int toyota_dbc_eps_torque_factor = 100;   // conversion factor for STEER_TORQUE_EPS in %: see dbc file

static uint32_t toyota_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint8_t checksum = (uint8_t)(msg->addr) + (uint8_t)((unsigned int)(msg->addr) >> 8U) + (uint8_t)(len);
  for (int i = 0; i < (len - 1); i++) {
    checksum += (uint8_t)msg->data[i];
  }
  return checksum;
}

static uint32_t toyota_get_checksum(const CANPacket_t *msg) {
  int checksum_byte = GET_LEN(msg) - 1U;
  return (uint8_t)(msg->data[checksum_byte]);
}

static bool toyota_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == 0x260U) {
    valid = !GET_BIT(msg, 3U);  // STEER_TORQUE_SENSOR.STEER_ANGLE_INITIALIZING
  }
  if (msg->addr == 0xaaU) {  // WHEEL_SPEEDS
    // each wheel speed is 1-bit fault + 15-bit speed
    valid = true;
    for (uint8_t i = 0U; i < 4U; i += 1U) {
      if (GET_BIT(msg, (i * 16U) + 7U)) {  // WHEEL_SPEED_xx_FAULT
        valid = false;
        break;
      }
    }
  }
  return valid;
}

static void toyota_rx_hook(const CANPacket_t *msg) {
  if (toyota_tss3_08a_host && !toyota_corolla_hf) {
    if ((msg->bus == 2U) && (msg->addr == 0x8AU) && (GET_LEN(msg) == 32U)) {
      for (uint8_t i = 0U; i < 28U; i++) {
        toyota_tss3_08a_native_app_prev2[i] = toyota_tss3_08a_native_app_prev1[i];
        toyota_tss3_08a_native_app_prev1[i] = toyota_tss3_08a_native_app[i];
        toyota_tss3_08a_native_app[i] = msg->data[i];
      }
      if (toyota_tss3_08a_native_history < 3U) {
        toyota_tss3_08a_native_history++;
      }
      toyota_tss3_08a_native_valid = msg->data[21] == 0U;
    }
    if ((msg->bus == 2U) && (msg->addr == 0xFU) && (GET_LEN(msg) == 8U)) {
      const uint32_t reset_counter = ((uint32_t)msg->data[2] << 12U) |
                                     ((uint32_t)msg->data[3] << 4U) |
                                     ((uint32_t)msg->data[4] >> 4U);
      if (toyota_tss3_08a_replacement_active && (reset_counter != toyota_tss3_08a_active_reset_counter)) {
        toyota_tss3_08a_replacement_active = false;
        toyota_tss3_08a_msg_low2_valid = false;
      }
      toyota_tss3_08a_reset_counter = reset_counter;
      toyota_tss3_08a_sync_valid = true;
    }
  }

  if (toyota_tss3_signer) {
    // Stock Toyota-B exposes the TSS3 EPS/Brake network on unsplit bus 1.
    if (msg_matches(msg, 0x25U, 1U)) {
      int angle_coarse = ((msg->data[0] & 0xFU) << 8U) | msg->data[1];
      angle_coarse = to_signed(angle_coarse, 12);
      const int angle_fraction = to_signed((msg->data[4] >> 4U) & 0xFU, 4);
      const int angle_tenths = (angle_coarse * 15) + angle_fraction;
      update_sample(&angle_meas, ROUND(((float)angle_tenths * 1787.0F) / 1024.0F));
    }

    if (msg_matches(msg, 0x116U, 1U)) {
      gas_pressed = msg->data[1] != 0U;
    }
    if (msg_matches(msg, 0x101U, 1U)) {
      brake_pressed = GET_BIT(msg, 3U);
    }
    if (msg_matches(msg, 0xAAU, 1U)) {
      int speed = 0;
      for (uint8_t i = 0U; i < 8U; i += 2U) {
        speed += (((msg->data[i] & 0x7FU) << 8U) | msg->data[i + 1U]) - 6767;
      }
      vehicle_moving = speed != 0;
      UPDATE_VEHICLE_SPEED(speed / 4.0 * 0.01 * KPH_TO_MS);
      if (toyota_tss3_08a_host && toyota_tss3_08a_replacement_active && vehicle_moving) {
        toyota_tss3_08a_replacement_active = false;
        toyota_tss3_08a_msg_low2_valid = false;
      }
    }
    if (!toyota_corolla_hf && msg_matches(msg, 0x8AU, 1U)) {
      pcm_cruise_check(GET_BIT(msg, 27U));
    }
    if (toyota_corolla_hf && msg_matches(msg, 0x8AU, 1U)) {
      pcm_cruise_check(GET_BIT(msg, 180U));
    }
    return;
  }

  // get eps motor torque (0.66 factor in dbc)
  if (msg_matches(msg, 0x260U, 0U)) {
    int torque_meas_new = (msg->data[5] << 8) | msg->data[6];
    torque_meas_new = to_signed(torque_meas_new, 16);

    // scale by dbc_factor
    torque_meas_new = (torque_meas_new * toyota_dbc_eps_torque_factor) / 100;

    // update array of sample
    update_sample(&torque_meas, torque_meas_new);

    // increase torque_meas by 1 to be conservative on rounding
    torque_meas.min--;
    torque_meas.max++;

    // driver torque for angle limiting
    int torque_driver_new = (msg->data[1] << 8) | msg->data[2];
    torque_driver_new = to_signed(torque_driver_new, 16);
    update_sample(&torque_driver, torque_driver_new);

    // LTA request angle should match current angle while inactive, clipped to max accepted angle.
    // note that angle can be relative to init angle on some TSS2 platforms, LTA has the same offset
    bool steer_angle_initializing = GET_BIT(msg, 3U);
    if (!steer_angle_initializing) {
      int angle_meas_new = (msg->data[3] << 8U) | msg->data[4];
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);
    }
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  // exit controls on rising edge of gas press, if not alternative experience
  // exit controls on rising edge of brake press
  if (toyota_secoc) {
    if (msg_matches(msg, 0x176U, 0U)) {
      bool cruise_engaged = GET_BIT(msg, 5U);  // PCM_CRUISE.CRUISE_ACTIVE
      pcm_cruise_check(cruise_engaged);
    }
    if (msg_matches(msg, 0x116U, 0U)) {
      gas_pressed = msg->data[1] != 0U;  // GAS_PEDAL.GAS_PEDAL_USER
    }
    if (msg_matches(msg, 0x101U, 0U)) {
      brake_pressed = GET_BIT(msg, 3U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_rav4_prime_generated.dbc)
    }
  } else {
    if (msg_matches(msg, 0x1D2U, 0U)) {
      bool cruise_engaged = GET_BIT(msg, 5U);  // PCM_CRUISE.CRUISE_ACTIVE
      pcm_cruise_check(cruise_engaged);
      gas_pressed = !GET_BIT(msg, 4U);  // PCM_CRUISE.GAS_RELEASED
    }
    if (!toyota_alt_brake && msg_matches(msg, 0x226U, 0U)) {
      brake_pressed = GET_BIT(msg, 37U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_nodsu_pt_generated.dbc)
    }
    if (toyota_alt_brake && msg_matches(msg, 0x224U, 0U)) {
      brake_pressed = GET_BIT(msg, 5U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_new_mc_pt_generated.dbc)
    }
  }

  // sample speed
  if (msg_matches(msg, 0xaaU, 0U)) {
    int speed = 0;
    // sum 4 wheel speeds. conversion: raw * 0.01 - 67.67
    for (uint8_t i = 0U; i < 8U; i += 2U) {
      int wheel_speed = ((msg->data[i] & 0x7FU) << 8U) | msg->data[(i + 1U)];
      speed += wheel_speed - 6767;
    }
    // check that all wheel speeds are at zero value
    vehicle_moving = speed != 0;

    UPDATE_VEHICLE_SPEED(speed / 4.0 * 0.01 * KPH_TO_MS);
  }
}

static bool toyota_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits TOYOTA_TORQUE_STEERING_LIMITS = {
    .max_torque = 1500,
    .max_rate_up = 15,          // ramp up slow
    .max_rate_down = 25,        // ramp down fast
    .max_torque_error = 350,    // max torque cmd in excess of motor torque
    .max_rt_delta = 450,        // the real time limit is 1800/sec, a 20% buffer
    .type = TorqueMotorLimited,

    // the EPS faults when the steering angle rate is above a certain threshold for too long. to prevent this,
    // we allow setting STEER_REQUEST bit to 0 while maintaining the requested torque value for a single frame
    .min_valid_request_frames = 17,
    .max_invalid_request_frames = 1,
    .min_valid_request_rt_interval = 162000,  // 162ms; a ~10% buffer on cutting every 18 frames
    .has_steer_req_tolerance = true,
  };

  static const AngleSteeringLimits TOYOTA_ANGLE_STEERING_LIMITS = {
    // LTA angle limits
    // factor for STEER_TORQUE_SENSOR->STEER_ANGLE and STEERING_LTA->STEER_ANGLE_CMD (1 / 0.0573)
    .max_angle = 1657,  // EPS only accepts up to 94.9461
    .angle_deg_to_can = 17.452007,
    .angle_rate_up_lookup = {
      {5., 25., 25.},
      {0.3, 0.15, 0.15}
    },
    .angle_rate_down_lookup = {
      {5., 25., 25.},
      {0.36, 0.26, 0.26}
    },
  };

  const int TOYOTA_LTA_MAX_MEAS_TORQUE = 1500;
  const int TOYOTA_LTA_MAX_DRIVER_TORQUE = 150;

  // longitudinal limits
  const LongitudinalLimits TOYOTA_LONG_LIMITS = {
    .max_accel = 2000,   // 2.0 m/s2
    .min_accel = -3500,  // -3.5 m/s2
  };

  bool tx = true;

  if (toyota_tss3_signer) {
    static const AngleSteeringLimits TOYOTA_TSS3_ANGLE_STEERING_LIMITS = {
      .max_angle = 1745,
      .angle_deg_to_can = 17.451171875F,
      .angle_rate_up_lookup = {
        {5., 25., 25.},
        {0.15, 0.075, 0.075}
      },
      .angle_rate_down_lookup = {
        {5., 25., 25.},
        {0.18, 0.13, 0.13}
      },
    };

    const bool signer_control = (msg->bus == 1U) && (msg->addr == 0x777U);
    const bool oracle_transport = toyota_tss3_08a_host && !toyota_corolla_hf &&
                                  (msg->bus == 1U) && (msg->addr == 0x7A1U);
    const bool host_08a = toyota_tss3_08a_host && !toyota_corolla_hf &&
                          (msg->bus == 0U) && (msg->addr == 0x8AU);
    const bool corolla_brake_cancel = toyota_corolla_hf && (msg->bus == 1U) && (msg->addr == 0x101U);
    tx = signer_control || oracle_transport || host_08a || corolla_brake_cancel;
    if (signer_control) {
      const bool c7_header_valid = !msg->fd && (msg->data[0] == 7U) && (msg->data[1] == 0xC7U) &&
                                    (msg->data[2] == 0xC7U) && (msg->data[6] == 0U) && (msg->data[7] == 0U);
      const bool host_admin_valid = toyota_tss3_08a_host && !toyota_corolla_hf && !msg->fd &&
                                    (msg->data[0] == 7U) && (msg->data[1] == 0xC9U) &&
                                    (msg->data[2] == 0xA8U) && (msg->data[3] <= 1U) &&
                                    (msg->data[5] == 0U) && (msg->data[6] == 0U) && (msg->data[7] == 0U);
      if (host_admin_valid) {
        const bool release = msg->data[3] == 0U;
        if (release) {
          tx = msg->data[4] == 0U;
          if (tx) {
            toyota_tss3_08a_replacement_active = false;
            toyota_tss3_08a_msg_low2_valid = false;
          }
        } else {
          const uint8_t expected_b26 = (toyota_tss3_08a_native_app[26] + 1U) & 0x3FU;
          tx = toyota_tss3_08a_native_valid && toyota_tss3_08a_sync_valid && !vehicle_moving &&
               (msg->data[4] == expected_b26);
          if (tx) {
            toyota_tss3_08a_next_b26 = expected_b26;
            toyota_tss3_08a_active_reset_counter = toyota_tss3_08a_reset_counter;
            toyota_tss3_08a_last_tx_ts = microsecond_timer_get();
            toyota_tss3_08a_msg_low2_valid = false;
            toyota_tss3_08a_replacement_active = true;
          }
        }
      } else {
        int target_angle = (msg->data[4] << 8U) | msg->data[5];
        target_angle = to_signed(target_angle, 16);
        const bool steer_control_enabled = msg->data[3] != 0U;
        tx = c7_header_valid && !safety_max_limit_check(target_angle, TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle,
                                                       -TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle) &&
                               !steer_angle_cmd_checks(target_angle, steer_control_enabled, TOYOTA_TSS3_ANGLE_STEERING_LIMITS);
      }
    }
    if (oracle_transport) {
      const bool first_frame = !msg->fd && (msg->data[0] == 0x10U) && (msg->data[1] == 40U) &&
                               (msg->data[2] == 0xC9U) && (msg->data[3] == 0xC9U) &&
                               (msg->data[5] == 0U) && (msg->data[6] == 0x8AU);
      const bool consecutive_frame = !msg->fd && (toyota_tss3_08a_oracle_next_cf != 0U) &&
                                     (msg->data[0] == (0x20U | toyota_tss3_08a_oracle_next_cf));
      tx = first_frame || consecutive_frame;
      if (first_frame) {
        toyota_tss3_08a_oracle_next_cf = 1U;
      } else if (consecutive_frame) {
        toyota_tss3_08a_oracle_next_cf++;
        if (toyota_tss3_08a_oracle_next_cf > 5U) {
          toyota_tss3_08a_oracle_next_cf = 0U;
        }
      }
    }
    if (host_08a) {
      bool application_matches_current = true;
      bool application_matches_prev1 = toyota_tss3_08a_signed && (toyota_tss3_08a_native_history >= 2U);
      bool application_matches_prev2 = toyota_tss3_08a_signed && (toyota_tss3_08a_native_history >= 3U);
      for (uint8_t i = 0U; i < 28U; i++) {
        if (i != 26U) {
          application_matches_current &= msg->data[i] == toyota_tss3_08a_native_app[i];
          application_matches_prev1 &= msg->data[i] == toyota_tss3_08a_native_app_prev1[i];
          application_matches_prev2 &= msg->data[i] == toyota_tss3_08a_native_app_prev2[i];
        }
      }
      bool application_matches = toyota_tss3_08a_replacement_active && toyota_tss3_08a_native_valid &&
                                 toyota_tss3_08a_sync_valid && msg->fd && (GET_LEN(msg) == 32U) &&
                                 (msg->data[21] == 0U) &&
                                 (application_matches_current || application_matches_prev1 || application_matches_prev2);
      const uint8_t b26 = msg->data[26] & 0x3FU;
      application_matches &= (msg->data[26] & 0xC0U) == (toyota_tss3_08a_native_app[26] & 0xC0U);
      application_matches &= b26 == toyota_tss3_08a_next_b26;
      const uint8_t fv4 = msg->data[28] >> 4U;
      const uint8_t reset_low2 = toyota_tss3_08a_reset_counter & 0x3U;
      const uint8_t message_low2 = (fv4 >> 2U) & 0x3U;
      application_matches &= (fv4 & 0x3U) == reset_low2;
      if (toyota_tss3_08a_msg_low2_valid) {
        application_matches &= message_low2 == ((toyota_tss3_08a_msg_low2 + 1U) & 0x3U);
      }
      tx = application_matches;
      if (tx) {
        toyota_tss3_08a_next_b26 = (toyota_tss3_08a_next_b26 + 1U) & 0x3FU;
        toyota_tss3_08a_msg_low2 = message_low2;
        toyota_tss3_08a_msg_low2_valid = true;
        toyota_tss3_08a_last_tx_ts = microsecond_timer_get();
      }
    }
    if (corolla_brake_cancel) {
      // Stock-longitudinal cancel follows the normal openpilot policy boundary:
      // require the native cancel actuation bit and a valid Toyota checksum.
      // CarController clones every non-cancel field from the live 0x101 frame.
      const bool brake_cancel = GET_BIT(msg, 3U);
      const bool checksum_valid = toyota_compute_checksum(msg) == toyota_get_checksum(msg);
      tx = brake_cancel && checksum_valid;
    }
    return tx;
  }

  // Check if msg is sent on BUS 0
  // ACCEL: safety check on byte 1-2
  if (msg_matches(msg, 0x343U, 0U)) {
    int desired_accel = (msg->data[0] << 8) | msg->data[1];
    desired_accel = to_signed(desired_accel, 16);

    bool violation = false;
    if (toyota_secoc) {
      // SecOC cars move accel to 0x183. Only allow inactive accel on 0x343 to match stock behavior
      violation = desired_accel != TOYOTA_LONG_LIMITS.inactive_accel;
    }
    violation |= longitudinal_accel_checks(desired_accel, TOYOTA_LONG_LIMITS);

    // only ACC messages that cancel are allowed when openpilot is not controlling longitudinal
    if (toyota_stock_longitudinal) {
      bool cancel_req = GET_BIT(msg, 24U);
      if (!cancel_req) {
        violation = true;
      }
      if (desired_accel != TOYOTA_LONG_LIMITS.inactive_accel) {
        violation = true;
      }
    }

    if (violation) {
      tx = false;
    }
  }

  if (msg_matches(msg, 0x183U, 0U)) {
    int desired_accel = (msg->data[0] << 8) | msg->data[1];
    desired_accel = to_signed(desired_accel, 16);

    tx = !longitudinal_accel_checks(desired_accel, TOYOTA_LONG_LIMITS);
  }

  // AEB: block all actuation. only used when DSU is unplugged
  if (msg_matches(msg, 0x283U, 0U)) {
    // only allow the checksum, which is the last byte
    bool block = GET_BYTES_64_LE(msg, 0, 6) != 0U;
    if (block) {
      tx = false;
    }
  }

  // STEERING_LTA angle steering check
  if (msg_matches(msg, 0x191U, 0U)) {
    // check the STEER_REQUEST, STEER_REQUEST_2, TORQUE_WIND_DOWN, STEER_ANGLE_CMD signals
    bool lta_request = GET_BIT(msg, 0U);
    bool lta_request2 = GET_BIT(msg, 25U);
    int torque_wind_down = msg->data[5];
    int lta_angle = (msg->data[1] << 8) | msg->data[2];
    lta_angle = to_signed(lta_angle, 16);

    bool steer_control_enabled = lta_request || lta_request2;
    if (!toyota_lta) {
      // using torque (LKA), block LTA msgs with actuation requests
      if (steer_control_enabled || (lta_angle != 0) || (torque_wind_down != 0)) {
        tx = false;
      }
    } else {
      // check angle rate limits and inactive angle
      if (steer_angle_cmd_checks(lta_angle, steer_control_enabled, TOYOTA_ANGLE_STEERING_LIMITS)) {
        tx = false;
      }

      if (lta_request != lta_request2) {
        tx = false;
      }

      // TORQUE_WIND_DOWN is gated on steer request
      if (!steer_control_enabled && (torque_wind_down != 0)) {
        tx = false;
      }

      // TORQUE_WIND_DOWN can only be no or full torque
      if ((torque_wind_down != 0) && (torque_wind_down != 100)) {
        tx = false;
      }

      // check if we should wind down torque
      int driver_torque = SAFETY_MIN(SAFETY_ABS(torque_driver.min), SAFETY_ABS(torque_driver.max));
      if ((driver_torque > TOYOTA_LTA_MAX_DRIVER_TORQUE) && (torque_wind_down != 0)) {
        tx = false;
      }

      int eps_torque = SAFETY_MIN(SAFETY_ABS(torque_meas.min), SAFETY_ABS(torque_meas.max));
      if ((eps_torque > TOYOTA_LTA_MAX_MEAS_TORQUE) && (torque_wind_down != 0)) {
        tx = false;
      }
    }
  }

  // STEERING_LTA_2 angle steering check (SecOC)
  if (toyota_secoc && msg_matches(msg, 0x131U, 0U)) {
    // SecOC cars block any form of LTA actuation for now
    bool lta_request = GET_BIT(msg, 3U);  // STEERING_LTA_2.STEER_REQUEST
    bool lta_request2 = GET_BIT(msg, 0U);  // STEERING_LTA_2.STEER_REQUEST_2
    int lta_angle_msb = msg->data[2];  // STEERING_LTA_2.STEER_ANGLE_CMD (MSB)
    int lta_angle_lsb = msg->data[3];  // STEERING_LTA_2.STEER_ANGLE_CMD (LSB)

    bool actuation = lta_request || lta_request2 || (lta_angle_msb != 0) || (lta_angle_lsb != 0);
    if (actuation) {
      tx = false;
    }
  }

  // STEER: safety check on bytes 2-3
  if (msg_matches(msg, 0x2E4U, 0U)) {
    int desired_torque = (msg->data[1] << 8) | msg->data[2];
    desired_torque = to_signed(desired_torque, 16);
    bool steer_req = GET_BIT(msg, 0U);
    // When using LTA (angle control), assert no actuation on LKA message
    if (!toyota_lta) {
      if (steer_torque_cmd_checks(desired_torque, steer_req, TOYOTA_TORQUE_STEERING_LIMITS)) {
        tx = false;
      }
    } else {
      if ((desired_torque != 0) || steer_req) {
        tx = false;
      }
    }
  }

  // UDS: Only tester present ("\x0F\x02\x3E\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (msg->addr == 0x750U) {
    // this address is sub-addressed. only allow tester present to radar (0xF)
    bool invalid_uds_msg = GET_BYTES_64_LE(msg, 0, 8) != 0x00000000003E020FULL;
    if (invalid_uds_msg) {
      tx = false;
    }
  }

  return tx;
}

static bool toyota_fwd_hook(int bus_num, int addr) {
  bool block = false;
  if (toyota_tss3_08a_host && !toyota_corolla_hf && toyota_tss3_08a_replacement_active &&
      (bus_num == 2) && (addr == 0x8A)) {
    const uint32_t elapsed = safety_get_ts_elapsed(microsecond_timer_get(), toyota_tss3_08a_last_tx_ts);
    if (elapsed > TOYOTA_TSS3_08A_REPLACEMENT_TIMEOUT_US) {
      toyota_tss3_08a_replacement_active = false;
      toyota_tss3_08a_msg_low2_valid = false;
    } else {
      block = true;
    }
  }
  return block;
}

static safety_config toyota_init(uint16_t param) {
  static const CanMsg TOYOTA_TX_MSGS[] = {
    TOYOTA_COMMON_TX_MSGS
  };

  static const CanMsg TOYOTA_SECOC_TX_MSGS[] = {
    TOYOTA_COMMON_SECOC_TX_MSGS
  };

  static const CanMsg TOYOTA_LONG_TX_MSGS[] = {
    TOYOTA_COMMON_LONG_TX_MSGS
  };

  static const CanMsg TOYOTA_SECOC_LONG_TX_MSGS[] = {
    TOYOTA_COMMON_SECOC_LONG_TX_MSGS
  };

  // safety param flags
  // first byte is for EPS factor, second is for flags
  const uint32_t TOYOTA_PARAM_OFFSET = 8U;
  const uint32_t TOYOTA_EPS_FACTOR = (1UL << TOYOTA_PARAM_OFFSET) - 1U;
  const uint32_t TOYOTA_PARAM_ALT_BRAKE = 1UL << TOYOTA_PARAM_OFFSET;
  const uint32_t TOYOTA_PARAM_STOCK_LONGITUDINAL = 2UL << TOYOTA_PARAM_OFFSET;
  const uint32_t TOYOTA_PARAM_LTA = 4UL << TOYOTA_PARAM_OFFSET;
  const uint32_t TOYOTA_PARAM_TSS3_SIGNER = 16UL << TOYOTA_PARAM_OFFSET;
  const uint32_t TOYOTA_PARAM_COROLLA_HF = 32UL << TOYOTA_PARAM_OFFSET;
  const uint32_t TOYOTA_PARAM_TSS3_08A_HOST = 64UL << TOYOTA_PARAM_OFFSET;
  const uint32_t TOYOTA_PARAM_TSS3_08A_SIGNED = 128UL << TOYOTA_PARAM_OFFSET;

#ifdef ALLOW_DEBUG
  const uint32_t TOYOTA_PARAM_SECOC = 8UL << TOYOTA_PARAM_OFFSET;
  toyota_secoc = GET_FLAG(param, TOYOTA_PARAM_SECOC);
#endif

  toyota_alt_brake = GET_FLAG(param, TOYOTA_PARAM_ALT_BRAKE);
  toyota_stock_longitudinal = GET_FLAG(param, TOYOTA_PARAM_STOCK_LONGITUDINAL);
  toyota_lta = GET_FLAG(param, TOYOTA_PARAM_LTA);
  toyota_tss3_signer = GET_FLAG(param, TOYOTA_PARAM_TSS3_SIGNER);
  toyota_corolla_hf = GET_FLAG(param, TOYOTA_PARAM_COROLLA_HF);
  toyota_tss3_08a_host = GET_FLAG(param, TOYOTA_PARAM_TSS3_08A_HOST);
  toyota_tss3_08a_signed = GET_FLAG(param, TOYOTA_PARAM_TSS3_08A_SIGNED);
  toyota_tss3_08a_replacement_active = false;
  toyota_tss3_08a_native_valid = false;
  toyota_tss3_08a_native_history = 0U;
  toyota_tss3_08a_sync_valid = false;
  toyota_tss3_08a_msg_low2_valid = false;
  toyota_tss3_08a_oracle_next_cf = 0U;
  toyota_dbc_eps_torque_factor = param & TOYOTA_EPS_FACTOR;

  safety_config ret;
  if (toyota_tss3_signer) {
    if (toyota_corolla_hf) {
      static const CanMsg toyota_corolla_tss3_tx_msgs[] = {
        {0x777, 1, 8, .check_relay = false},
        {0x101, 1, 8, .check_relay = false},
      };
      SET_TX_MSGS(toyota_corolla_tss3_tx_msgs, ret);
      static RxCheck toyota_corolla_hf_rx_checks[] = {
        {.msg = {{0x025, 1, 32, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x0AA, 1, 8, 100U, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
        {.msg = {{0x116, 1, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x101, 1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x08A, 1, 32, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
      };
      SET_RX_CHECKS(toyota_corolla_hf_rx_checks, ret);
    } else {
      static const CanMsg toyota_f33_tss3_tx_msgs[] = {
        {0x777, 1, 8, .check_relay = false},
        {0x7A1, 1, 8, .check_relay = false},
        {0x08A, 0, 32, .check_relay = false},
      };
      SET_TX_MSGS(toyota_f33_tss3_tx_msgs, ret);
      static RxCheck toyota_f33_rx_checks[] = {
        {.msg = {{0x025, 1, 32, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x0AA, 1, 8, 100U, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
        {.msg = {{0x116, 1, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x101, 1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x08A, 1, 32, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
      };
      static RxCheck toyota_f33_08a_host_rx_checks[] = {
        {.msg = {{0x025, 1, 32, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x0AA, 1, 8, 100U, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
        {.msg = {{0x116, 1, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x101, 1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x08A, 1, 32, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x08A, 2, 32, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x00F, 2, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
      };
      if (toyota_tss3_08a_host) {
        SET_RX_CHECKS(toyota_f33_08a_host_rx_checks, ret);
      } else {
        SET_RX_CHECKS(toyota_f33_rx_checks, ret);
      }
    }
  } else if (toyota_secoc) {
    if (toyota_stock_longitudinal) {
      SET_TX_MSGS(TOYOTA_SECOC_TX_MSGS, ret);
    } else {
      SET_TX_MSGS(TOYOTA_SECOC_LONG_TX_MSGS, ret);
    }
  } else {
    if (toyota_stock_longitudinal) {
      SET_TX_MSGS(TOYOTA_TX_MSGS, ret);
    } else {
      SET_TX_MSGS(TOYOTA_LONG_TX_MSGS, ret);
    }
  }

  if (toyota_tss3_signer) {
    // TSS3 resident-signer checks were selected above.
  } else if (toyota_secoc) {
    static RxCheck toyota_secoc_rx_checks[] = {
      TOYOTA_SECOC_RX_CHECKS
    };

    SET_RX_CHECKS(toyota_secoc_rx_checks, ret);
  } else if (toyota_lta) {
    // Check the quality flag for angle measurement when using LTA, since it's not set on TSS-P cars
    static RxCheck toyota_lta_rx_checks[] = {
      TOYOTA_RX_CHECKS(true)
    };

    SET_RX_CHECKS(toyota_lta_rx_checks, ret);
  } else {
    static RxCheck toyota_lka_rx_checks[] = {
      TOYOTA_RX_CHECKS(false)
    };
    static RxCheck toyota_lka_alt_brake_rx_checks[] = {
      TOYOTA_ALT_BRAKE_RX_CHECKS(false)
    };

    if (!toyota_alt_brake) {
      SET_RX_CHECKS(toyota_lka_rx_checks, ret);
    } else {
      SET_RX_CHECKS(toyota_lka_alt_brake_rx_checks, ret);
    }
  }

  return ret;
}

const safety_hooks toyota_hooks = {
  .init = toyota_init,
  .rx = toyota_rx_hook,
  .tx = toyota_tx_hook,
  .fwd = toyota_fwd_hook,
  .get_checksum = toyota_get_checksum,
  .compute_checksum = toyota_compute_checksum,
  .get_quality_flag_valid = toyota_get_quality_flag_valid,
};
