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
static bool toyota_tss3_08a_host = false;
static bool toyota_tss3_08a_replacement_active = false;
static uint32_t toyota_tss3_08a_last_publication_ts = 0U;
static uint32_t toyota_tss3_08a_last_angle_check_ts = 0U;
#define TOYOTA_TSS3_08A_STOCK_HISTORY_LEN 16U
#define TOYOTA_TSS3_08A_APPLICATION_LEN 28U
#define TOYOTA_TSS3_08A_REPLACEMENT_TIMEOUT_US 100000U
static uint8_t toyota_tss3_08a_stock_applications[TOYOTA_TSS3_08A_STOCK_HISTORY_LEN][TOYOTA_TSS3_08A_APPLICATION_LEN] = {{0}};
static uint32_t toyota_tss3_08a_stock_timestamps[TOYOTA_TSS3_08A_STOCK_HISTORY_LEN] = {0};
static uint8_t toyota_tss3_08a_stock_history_next = 0U;
static uint8_t toyota_tss3_08a_stock_history_count = 0U;
static uint32_t toyota_tss3_08a_native_last_rx_ts = 0U;
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
  if (toyota_tss3_08a_host) {
    if ((msg->bus == 2U) && (msg->addr == 0x8AU) && (GET_LEN(msg) == 32U)) {
      for (uint8_t i = 0U; i < TOYOTA_TSS3_08A_APPLICATION_LEN; i++) {
        toyota_tss3_08a_stock_applications[toyota_tss3_08a_stock_history_next][i] = msg->data[i];
      }
      toyota_tss3_08a_stock_timestamps[toyota_tss3_08a_stock_history_next] = microsecond_timer_get();
      toyota_tss3_08a_stock_history_next = (toyota_tss3_08a_stock_history_next + 1U) % TOYOTA_TSS3_08A_STOCK_HISTORY_LEN;
      if (toyota_tss3_08a_stock_history_count < TOYOTA_TSS3_08A_STOCK_HISTORY_LEN) {
        toyota_tss3_08a_stock_history_count++;
      }
      toyota_tss3_08a_native_last_rx_ts = microsecond_timer_get();

      // The FRC operating latch remains the ordinary engagement input. The
      // recent application history also bounds stock-longitudinal replacement
      // to values received from the FRC while allowing signer pipeline delay.
      pcm_cruise_check(GET_BIT(msg, 27U));
    }
  }

  if (toyota_tss3_signer) {
    // Canonical repinned TSS3 chassis observations are on bus 0.
    const uint8_t tss3_state_bus = 0U;
    if (msg_matches(msg, 0x25U, tss3_state_bus)) {
      int angle_coarse = ((msg->data[0] & 0xFU) << 8U) | msg->data[1];
      angle_coarse = to_signed(angle_coarse, 12);
      const int angle_fraction = to_signed((msg->data[4] >> 4U) & 0xFU, 4);
      const int angle_tenths = (angle_coarse * 15) + angle_fraction;
      update_sample(&angle_meas, ROUND(((float)angle_tenths * 1787.0F) / 1024.0F));
    }

    if (msg_matches(msg, 0x116U, tss3_state_bus)) {
      gas_pressed = msg->data[1] != 0U;
    }
    if (msg_matches(msg, 0x101U, tss3_state_bus)) {
      brake_pressed = GET_BIT(msg, 3U);
    }
    if (msg_matches(msg, 0xAAU, tss3_state_bus)) {
      int speed = 0;
      for (uint8_t i = 0U; i < 8U; i += 2U) {
        const int wheel_speed = ((msg->data[i] & 0x7FU) << 8U) | msg->data[i + 1U];
        speed += wheel_speed - 6767;
      }
      vehicle_moving = speed != 0;
      UPDATE_VEHICLE_SPEED(speed / 4.0 * 0.01 * KPH_TO_MS);
    }
  } else {

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
      .frequency = 100U,
    };

    static const AngleSteeringParams TOYOTA_TSS3_STEERING_PARAMS = {
      .slip_factor = -0.0007484283457339188F,  // calc_slip_factor(VM)
      .steer_ratio = 15.3F,
      .wheelbase = 2.825F,
    };

    const bool signer_control = (msg->bus == 1U) && (msg->addr == 0x777U);
    // The EPS resident owns oracle sequencing and content validation. Panda
    // admits only the exact stateless fragment envelope; the signed 0x08A
    // remains the safety boundary for all resulting actuation.
    const bool oracle_transport = toyota_tss3_08a_host && !msg->fd &&
                                  (GET_LEN(msg) == 8U) && (msg->bus == 0U) && (msg->addr == 0x777U) &&
                                  ((msg->data[0] & 0xC0U) == 0x80U);
    const bool host_08a = toyota_tss3_08a_host &&
                          (msg->bus == 0U) && (msg->addr == 0x8AU);
    const bool camry_brake_cancel = toyota_tss3_08a_host &&
                                    (msg->bus == 2U) && (msg->addr == 0x101U);
    const bool camry_hud = toyota_tss3_08a_host && !msg->fd &&
                           (msg->bus == 0U) && (msg->addr == 0x412U);
    tx = signer_control || oracle_transport || host_08a || camry_brake_cancel || camry_hud;
    if (signer_control) {
      const bool host_admin_valid = toyota_tss3_08a_host && !msg->fd &&
                                    (msg->data[0] == 7U) && (msg->data[1] == 0xC9U) &&
                                    (msg->data[2] == 0xA8U) && (msg->data[3] <= 1U) &&
                                    (msg->data[4] == 0U) && (msg->data[5] == 0U) &&
                                    (msg->data[6] == 0U) && (msg->data[7] == 0U);
      if (host_admin_valid) {
        const bool release = msg->data[3] == 0U;
        if (release) {
          tx = true;
          toyota_tss3_08a_replacement_active = false;
        } else {
          // Arm switches 0x08A publication ownership. The EPS resident supplies
          // an independently fresh security trailer for every host frame.
          const uint32_t native_age = safety_get_ts_elapsed(microsecond_timer_get(), toyota_tss3_08a_native_last_rx_ts);
          tx = !toyota_stock_longitudinal || ((toyota_tss3_08a_stock_history_count > 0U) &&
                                               (native_age <= TOYOTA_TSS3_08A_REPLACEMENT_TIMEOUT_US));
          if (tx) {
            toyota_tss3_08a_replacement_active = true;
            toyota_tss3_08a_last_publication_ts = microsecond_timer_get();
            // Admin and the first replacement are emitted in one send batch.
            // Give that first frame one nominal 100 Hz interval of rate budget.
            toyota_tss3_08a_last_angle_check_ts = microsecond_timer_get() - 10000U;
            desired_angle_last = SAFETY_CLAMP(angle_meas.values[0],
                                              -TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle,
                                               TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle);
          }
        }
      } else {
        // The retired direct-steering sideband is never accepted.
        tx = false;
      }
    }
    if (host_08a) {
      tx = false;
      const uint32_t now = microsecond_timer_get();
      bool application_shape = toyota_tss3_08a_replacement_active &&
                               msg->fd && (GET_LEN(msg) == 32U);
      bool actuation_valid = true;

      if (application_shape && toyota_stock_longitudinal) {
        bool stock_application_match = false;
        for (uint8_t history_index = 0U; history_index < toyota_tss3_08a_stock_history_count; history_index++) {
          bool candidate_match = safety_get_ts_elapsed(now, toyota_tss3_08a_stock_timestamps[history_index]) <=
                                 TOYOTA_TSS3_08A_REPLACEMENT_TIMEOUT_US;
          for (uint8_t i = 0U; i < TOYOTA_TSS3_08A_APPLICATION_LEN; i++) {
            uint8_t compare_mask = 0xFFU;
            if ((i == 18U) || (i == 19U) || (i == 24U) || (i == 25U)) {
              compare_mask = 0U;
            } else if ((i == 21U) || (i == 26U)) {
              compare_mask = 0xC0U;
            }
            candidate_match &= ((msg->data[i] ^ toyota_tss3_08a_stock_applications[history_index][i]) & compare_mask) == 0U;
          }
          stock_application_match |= candidate_match;
        }
        application_shape &= stock_application_match;
      } else if (application_shape) {
        // Complete comma-owned application envelope. Only set speed, commanded
        // angle/acceleration, request sequence, and MAC are variable.
        application_shape &= (msg->data[0] == 0U) && (msg->data[1] == 0U) &&
                             (msg->data[2] == 0U) && (msg->data[3] == 0x08U) &&
                             (msg->data[4] == 0x80U) && (msg->data[5] == 0U) &&
                             (msg->data[13] == 0x7FU) && (msg->data[14] == 0xFFU) &&
                             (msg->data[15] == 0U) && (msg->data[16] == 0x7FU) &&
                             (msg->data[17] == 0xFFU) && (msg->data[20] == 0xC0U) &&
                             ((msg->data[21] & 0xC0U) == 0U) &&
                             (msg->data[22] == 0x10U) && (msg->data[23] == 0U) &&
                             (msg->data[25] == 0U) && (msg->data[27] == 0U);
      }

      if (application_shape) {
        const uint8_t host_id = msg->data[21] & 0x3FU;
        const bool host_lateral_active = (host_id == 11U) && (msg->data[24] == 100U);
        const bool host_lateral_inactive = (host_id == 0U) && (msg->data[24] == 50U);
        application_shape &= (host_lateral_active || host_lateral_inactive) && (msg->data[25] == 0U);

        int target_angle = (msg->data[18] << 8U) | msg->data[19];
        target_angle = to_signed(target_angle, 16);
        bool angle_violation = false;
        // CarController creates one angle-limited generation per nominal 10 ms
        // control tick. Signer and CAN scheduling jitter can publish adjacent
        // completed generations less than 10 ms apart, but that must not shrink
        // the budget already applied when the command was generated. Longer
        // publication gaps still receive their actual elapsed-time budget.
        const uint32_t nominal_generation_interval_us = 1000000U / TOYOTA_TSS3_ANGLE_STEERING_LIMITS.frequency;
        const uint32_t angle_rate_interval_us = SAFETY_MAX(
          safety_get_ts_elapsed(now, toyota_tss3_08a_last_angle_check_ts), nominal_generation_interval_us
        );
        if (host_lateral_active) {
          angle_violation = steer_angle_cmd_checks_vm_timed(target_angle, true, TOYOTA_TSS3_ANGLE_STEERING_LIMITS,
                                                             TOYOTA_TSS3_STEERING_PARAMS, angle_rate_interval_us);
        } else if (host_lateral_inactive) {
          angle_violation = steer_angle_cmd_checks_vm_timed(target_angle, false, TOYOTA_TSS3_ANGLE_STEERING_LIMITS,
                                                             TOYOTA_TSS3_STEERING_PARAMS, angle_rate_interval_us);
        } else {
          // The application-shape check above rejects every other lateral ID.
        }
        if (host_lateral_active || host_lateral_inactive) {
          toyota_tss3_08a_last_angle_check_ts = now;
        }
        const bool max_angle_violation = safety_max_limit_check(target_angle, TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle,
                                                                -TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle);
        angle_violation |= max_angle_violation;
        if (max_angle_violation) {
          desired_angle_last = SAFETY_CLAMP(angle_meas.values[0], -TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle,
                                            TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle);
        }
        actuation_valid &= !angle_violation;

        if (!toyota_stock_longitudinal) {
          int accel_a = (msg->data[8] << 8U) | msg->data[9];
          int accel_b = (msg->data[11] << 8U) | msg->data[12];
          accel_a = to_signed(accel_a, 16);
          accel_b = to_signed(accel_b, 16);
          const bool host_longitudinal = (msg->data[6] == 0x2DU) && (msg->data[7] == 0x47U) &&
                                         (accel_a == accel_b);
          application_shape &= host_longitudinal;
          // The VMC arbitration layer owns driver override and publishes the
          // selected longitudinal result as ID 63. Retain the normal controls
          // gate and absolute request envelope, but do not apply Panda's
          // conventional gas-pedal veto to the pre-arbitration request.
          const bool accel_valid = controls_allowed &&
                                   !safety_max_limit_check(accel_a, TOYOTA_LONG_LIMITS.max_accel,
                                                           TOYOTA_LONG_LIMITS.min_accel);
          const bool accel_inactive = accel_a == TOYOTA_LONG_LIMITS.inactive_accel;
          actuation_valid &= (accel_valid || accel_inactive) && host_longitudinal;
        }
      }

      if (application_shape) {
        tx = actuation_valid;
        if (tx) {
          // Rejected applications do not renew replacement ownership. This
          // keeps a safety rejection from suppressing stock traffic forever.
          toyota_tss3_08a_last_publication_ts = now;
        }
      }
    }
    if (camry_brake_cancel) {
      // Stock-ACC cancel is ordinary Brake Module traffic with only the native
      // brake/cancel bit asserted. Exact Camry road data bounds the stable zero
      // companions; B1/B3 remain live cloned fields and checksum must be valid.
      const bool brake_cancel = GET_BIT(msg, 3U);
      const bool checksum_valid = toyota_compute_checksum(msg) == toyota_get_checksum(msg);
      const bool camry_shape = !camry_brake_cancel || ((msg->data[0] == 0x88U) &&
                               (msg->data[2] == 0U) && (msg->data[4] == 0U) &&
                               (msg->data[5] == 0U) && (msg->data[6] == 0U));
      tx = brake_cancel && checksum_valid && camry_shape;
    }
  } else {

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
  }

  return tx;
}

static bool toyota_fwd_hook(int bus_num, int addr) {
  // Fail open if authenticated host replacement traffic disappears for 100 ms.
  bool block = toyota_tss3_08a_host &&
               (bus_num == 2) && (addr == 0x412);
  if (toyota_tss3_08a_host && toyota_tss3_08a_replacement_active &&
      (bus_num == 2) && (addr == 0x8A)) {
    const uint32_t elapsed = safety_get_ts_elapsed(microsecond_timer_get(), toyota_tss3_08a_last_publication_ts);
    if (elapsed > TOYOTA_TSS3_08A_REPLACEMENT_TIMEOUT_US) {
      toyota_tss3_08a_replacement_active = false;
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
  const uint32_t TOYOTA_PARAM_TSS3_08A_HOST = 64UL << TOYOTA_PARAM_OFFSET;

#ifdef ALLOW_DEBUG
  const uint32_t TOYOTA_PARAM_SECOC = 8UL << TOYOTA_PARAM_OFFSET;
  toyota_secoc = GET_FLAG(param, TOYOTA_PARAM_SECOC);
#endif

  toyota_alt_brake = GET_FLAG(param, TOYOTA_PARAM_ALT_BRAKE);
  toyota_stock_longitudinal = GET_FLAG(param, TOYOTA_PARAM_STOCK_LONGITUDINAL);
  toyota_lta = GET_FLAG(param, TOYOTA_PARAM_LTA);
  toyota_tss3_signer = GET_FLAG(param, TOYOTA_PARAM_TSS3_SIGNER);
  toyota_tss3_08a_host = GET_FLAG(param, TOYOTA_PARAM_TSS3_08A_HOST);
  toyota_tss3_08a_replacement_active = false;
  toyota_tss3_08a_last_publication_ts = 0U;
  toyota_tss3_08a_last_angle_check_ts = 0U;
  toyota_tss3_08a_stock_history_next = 0U;
  toyota_tss3_08a_stock_history_count = 0U;
  toyota_tss3_08a_native_last_rx_ts = 0U;
  toyota_dbc_eps_torque_factor = param & TOYOTA_EPS_FACTOR;

  safety_config ret;
  if (toyota_tss3_signer) {
    static const CanMsg toyota_tss3_tx_msgs[] = {
      {0x777, 1, 8, .check_relay = false},
      {0x777, 0, 8, .check_relay = false},
      {0x08A, 0, 32, .check_relay = true, .disable_static_blocking = true},
      {0x101, 2, 8, .check_relay = false},
      // Only the relay topology replaces 0x412. Keep relay collision checking,
      // while the dynamic forwarding hook selects that physical topology.
      {0x412, 0, 8, .check_relay = true, .disable_static_blocking = true},
    };
    SET_TX_MSGS(toyota_tss3_tx_msgs, ret);
    static RxCheck toyota_tss3_rx_checks[] = {
      {.msg = {{0x025, 0, 32, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
      {.msg = {{0x0AA, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
      {.msg = {{0x116, 0, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
      {.msg = {{0x101, 0, 8, 50U, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
      // Native 0x08A remains an FRC-presence and engagement-state input only.
      {.msg = {{0x08A, 2, 32, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    };
    SET_RX_CHECKS(toyota_tss3_rx_checks, ret);
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
