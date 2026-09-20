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

#define TOYOTA_PCM_CRUISE_2_ADDR_CHECK                                                                                                     \
  {.msg = {{0x1D3, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 33U}, { 0 }, { 0 }}},  \

#define TOYOTA_DSU_CRUISE_ADDR_CHECK                                                                                                      \
  {.msg = {{0x365, 0, 7, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 5U, .ignore_frequency_check = true}, { 0 }, { 0 }}},  \

#define TOYOTA_GAS_INTERCEPTOR_ADDR_CHECK                                                   \
  {.msg = {{0x201, 0, 6, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

static bool toyota_secoc = false;
static bool toyota_alt_brake = false;
static bool toyota_stock_longitudinal = false;
static bool toyota_lta = false;
static bool toyota_tss3_signer = false;
static bool toyota_corolla_hf = false;
static bool toyota_tss3_08a_host = false;
static bool toyota_tss3_08a_replacement_active = false;
#define TOYOTA_TSS3_08A_NATIVE_HISTORY_LEN 4U
static uint8_t toyota_tss3_08a_native_frames[TOYOTA_TSS3_08A_NATIVE_HISTORY_LEN][32] = {{0}};
static bool toyota_tss3_08a_native_consumed[TOYOTA_TSS3_08A_NATIVE_HISTORY_LEN] = {false};
static uint8_t toyota_tss3_08a_native_history = 0U;
static uint32_t toyota_tss3_08a_native_last_rx_ts = 0U;
static uint32_t toyota_tss3_08a_last_tx_ts = 0U;
static uint8_t toyota_tss3_08a_oracle_seq = 0U;
static uint8_t toyota_tss3_08a_oracle_next_fragment = 0U;
static bool toyota_tss3_08a_first_host_frame = false;

// Native 0x08A is ~40 Hz with observed ~20-34 ms source intervals. Four source
// generations cover normal EPS oracle response latency without retaining stale
// host work. Fail open if host replacement traffic disappears for 100 ms.
const uint32_t TOYOTA_TSS3_08A_REPLACEMENT_TIMEOUT_US = 100000U;
static int toyota_dbc_eps_torque_factor = 100;   // conversion factor for STEER_TORQUE_EPS in %: see dbc file

static uint32_t toyota_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint8_t checksum = (uint8_t)(msg->addr) + (uint8_t)((unsigned int)(msg->addr) >> 8U) + (uint8_t)(len);
  for (int i = 0; i < (len - 1); i++) {
    checksum += (uint8_t)msg->data[i];
  }
  return checksum;
}

static bool toyota_f33_angle_cmd_checks(int desired_angle, bool steer_control_enabled,
                                        const AngleSteeringLimits limits, const AngleSteeringParams params) {
  const int desired_angle_previous = desired_angle_last;
  bool conditioner_violation = false;
  if (controls_allowed && steer_control_enabled) {
    // Exact F33 conditions 7 doubled-domain units per 5 ms foreground call.
    // This is 7 B6 counts per 10 ms openpilot tick. The request-plane proxy
    // can legitimately sample up to four controller ticks on one native 0x08A
    // generation, while the direct resident-signer sideband samples every tick.
    const uint32_t delta_frames = SAFETY_MAX(limits.angle_rate_delta_frames, 1U);
    const int max_conditioned_delta = 7 * delta_frames;
    conditioner_violation = safety_max_limit_check(desired_angle,
                                                   desired_angle_previous + max_conditioned_delta,
                                                   desired_angle_previous - max_conditioned_delta);
  }

  bool violation = safety_max_limit_check(desired_angle, limits.max_angle, -limits.max_angle);
  violation |= steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, limits, params);
  violation |= conditioner_violation;
  if (violation) {
    desired_angle_last = SAFETY_CLAMP(angle_meas.values[0], -limits.max_angle, limits.max_angle);
  }
  return violation;
}

static uint32_t toyota_get_checksum(const CANPacket_t *msg) {
  int checksum_byte = GET_LEN(msg) - 1U;
  return (uint8_t)(msg->data[checksum_byte]);
}

static bool toyota_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == 0x260U) {
    valid = !GET_BIT(msg, 3U);  // STEER_TORQUE_SENSOR.STEER_ANGLE_INITIALIZING
  } else if (msg->addr == 0xaaU) {  // WHEEL_SPEEDS
    // each wheel speed is 1-bit fault + 15-bit speed
    valid = true;
    for (uint8_t i = 0U; i < 4U; i += 1U) {
      if (GET_BIT(msg, (i * 16U) + 7U)) {  // WHEEL_SPEED_xx_FAULT
        valid = false;
        break;
      }
    }
  } else {
  }
  return valid;
}

static int TOYOTA_GET_INTERCEPTOR(const CANPacket_t *msg) {
  uint16_t val1 = (uint16_t)((uint16_t)msg->data[0] << 8U) | (uint16_t)msg->data[1];
  uint16_t val2 = (uint16_t)((uint16_t)msg->data[2] << 8U) | (uint16_t)msg->data[3];
  uint16_t avg  = (uint16_t)((val1 + val2) / 2U);

  return (int)avg;
}

static void toyota_rx_hook(const CANPacket_t *msg) {
  if (toyota_tss3_08a_host && !toyota_corolla_hf) {
    if ((msg->bus == 2U) && (msg->addr == 0x8AU) && (GET_LEN(msg) == 32U)) {
      for (uint8_t history_index = TOYOTA_TSS3_08A_NATIVE_HISTORY_LEN - 1U; history_index > 0U; history_index--) {
        for (uint8_t i = 0U; i < 32U; i++) {
          toyota_tss3_08a_native_frames[history_index][i] = toyota_tss3_08a_native_frames[history_index - 1U][i];
        }
        toyota_tss3_08a_native_consumed[history_index] = toyota_tss3_08a_native_consumed[history_index - 1U];
      }
      for (uint8_t i = 0U; i < 32U; i++) {
        toyota_tss3_08a_native_frames[0][i] = msg->data[i];
      }
      toyota_tss3_08a_native_consumed[0] = false;
      if (toyota_tss3_08a_native_history < TOYOTA_TSS3_08A_NATIVE_HISTORY_LEN) {
        toyota_tss3_08a_native_history++;
      }
      if (toyota_tss3_08a_replacement_active && toyota_tss3_08a_first_host_frame) {
        // If a newer source generation arrives before the atomic handoff clone,
        // the previously latest pre-arm generation is no longer eligible.
        // Keep only the newest native generation available as the handoff witness.
        for (uint8_t history_index = 1U; history_index < toyota_tss3_08a_native_history; history_index++) {
          toyota_tss3_08a_native_consumed[history_index] = true;
        }
      }
      toyota_tss3_08a_native_last_rx_ts = microsecond_timer_get();
      // FRC remains the native engagement owner with both stock and openpilot
      // longitudinal. Follow the same source-real operating latch consumed by
      // CarState so Panda and selfdrived cannot develop independent states.
      pcm_cruise_check(GET_BIT(msg, 27U));
    }
  }

  if (toyota_tss3_signer) {
    // Stock Toyota-B uses unsplit bus 1. Exact-F33 host replacement requires
    // the physical repin onto Panda's 0<->2 relay pair, with chassis state
    // consumed downstream on bus 0 and the FRC source observed upstream on 2.
    const uint8_t tss3_state_bus = toyota_tss3_08a_host ? 0U : 1U;
    if (msg->bus == tss3_state_bus) {
      if (msg->addr == 0x25U) {
        int angle_coarse = ((msg->data[0] & 0xFU) << 8U) | msg->data[1];
        angle_coarse = to_signed(angle_coarse, 12);
        const int angle_fraction = to_signed((msg->data[4] >> 4U) & 0xFU, 4);
        const int angle_tenths = (angle_coarse * 15) + angle_fraction;
        update_sample(&angle_meas, ROUND(((float)angle_tenths * 1787.0F) / 1024.0F));
      }

      if (msg->addr == 0x116U) {
        gas_pressed = msg->data[1] != 0U;
      }
      if (msg->addr == 0x101U) {
        brake_pressed = GET_BIT(msg, 3U);
      }
      if (msg->addr == 0xAAU) {
        int speed = 0;
        for (uint8_t i = 0U; i < 8U; i += 2U) {
          speed += (((msg->data[i] & 0x7FU) << 8U) | msg->data[i + 1U]) - 6767;
        }
        vehicle_moving = speed != 0;
        UPDATE_VEHICLE_SPEED(speed / 4.0 * 0.01 * KPH_TO_MS);
      }
      if (!toyota_corolla_hf && !toyota_tss3_08a_host && (msg->addr == 0x8AU)) {
        pcm_cruise_check(GET_BIT(msg, 27U));
      }
      if (toyota_corolla_hf && (msg->addr == 0x8AU)) {
        pcm_cruise_check(GET_BIT(msg, 180U));
      }
    }
  } else if (msg->bus == 0U) {

    // get eps motor torque (0.66 factor in dbc)
    if (msg->addr == 0x260U) {
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
      if (msg->addr == 0x176U) {
        bool cruise_engaged = GET_BIT(msg, 5U);  // PCM_CRUISE.CRUISE_ACTIVE
        pcm_cruise_check(cruise_engaged);
      }
      if (msg->addr == 0x116U) {
        gas_pressed = msg->data[1] != 0U;  // GAS_PEDAL.GAS_PEDAL_USER
      }
      if (msg->addr == 0x101U) {
        brake_pressed = GET_BIT(msg, 3U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_rav4_prime_generated.dbc)
      }
    } else {
      if (msg->addr == 0x1D2U) {
        bool cruise_engaged = GET_BIT(msg, 5U);  // PCM_CRUISE.CRUISE_ACTIVE
        pcm_cruise_check(cruise_engaged);

        if (!enable_gas_interceptor) {
          gas_pressed = !GET_BIT(msg, 4U);  // PCM_CRUISE.GAS_RELEASED
        }
      }
      if (!toyota_alt_brake && (msg->addr == 0x226U)) {
        brake_pressed = GET_BIT(msg, 37U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_nodsu_pt_generated.dbc)
      }
      if (toyota_alt_brake && (msg->addr == 0x224U)) {
        brake_pressed = GET_BIT(msg, 5U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_new_mc_pt_generated.dbc)
      }
    }

    // sample speed
    if (msg->addr == 0xaaU) {
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

    if (msg->addr == 0x1D3U) {
      acc_main_on = GET_BIT(msg, 15U);
    }

    if (msg->addr == 0x365U) {
      acc_main_on = GET_BIT(msg, 0U);
    }

    // sample gas interceptor
    if (msg->addr == 0x201U) {
      // panda interceptor threshold needs to be equivalent to openpilot threshold to avoid controls mismatches
      // If thresholds are mismatched then it is possible for panda to see the gas fall and rise while openpilot is in the pre-enabled state
      // Threshold calculated from DBC gains: round((((15 + 75.555) / 0.159375) + ((15 + 151.111) / 0.159375)) / 2) = 805
      const int toyota_gas_interceptor_thrsld = 805;

      int gas_interceptor = TOYOTA_GET_INTERCEPTOR(msg);
      gas_pressed = gas_interceptor > toyota_gas_interceptor_thrsld;

      gas_interceptor_prev = gas_interceptor;
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

  const LongitudinalLimits TOYOTA_F33_LONG_LIMITS = {
    .max_accel = 1300,   // 1.3 m/s2
    .min_accel = -1500,  // -1.5 m/s2
  };

  bool tx = true;

  if (toyota_tss3_signer) {
    static const AngleSteeringLimits TOYOTA_TSS3_COROLLA_ANGLE_STEERING_LIMITS = {
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

    static const AngleSteeringLimits TOYOTA_F33_ANGLE_STEERING_LIMITS = {
      .max_angle = 1745,
      .angle_deg_to_can = 17.451171875F,
      .frequency = 100U,
      .angle_rate_delta_frames = 1U,
    };

    static const AngleSteeringLimits TOYOTA_F33_08A_ANGLE_STEERING_LIMITS = {
      .max_angle = 1745,
      .angle_deg_to_can = 17.451171875F,
      .frequency = 100U,
      // Native 0x08A source intervals can straddle four 100-Hz controller
      // updates. CarController remains the tighter per-tick authority.
      .angle_rate_delta_frames = 4U,
    };

    // Exact 2026 Camry parameters used by CarController's VehicleModel.
    static const AngleSteeringParams TOYOTA_F33_ANGLE_STEERING_PARAMS = {
      .slip_factor = -0.0007485661713436738F,
      .steer_ratio = 15.3F,
      .wheelbase = 2.8244800567626953F,
    };

    const bool signer_control = (msg->bus == 1U) && (msg->addr == 0x777U);
    const bool oracle_transport = toyota_tss3_08a_host && !toyota_corolla_hf &&
                                  (msg->bus == 0U) && (msg->addr == 0x1FDC0002U);
    const bool host_08a = toyota_tss3_08a_host && !toyota_corolla_hf &&
                          (msg->bus == 0U) && (msg->addr == 0x8AU);
    const bool corolla_brake_cancel = toyota_corolla_hf && (msg->bus == 1U) && (msg->addr == 0x101U);
    const bool camry_brake_cancel = toyota_tss3_08a_host && !toyota_corolla_hf &&
                                    (msg->bus == 2U) && (msg->addr == 0x101U);
    tx = signer_control || oracle_transport || host_08a || corolla_brake_cancel || camry_brake_cancel;
    if (signer_control) {
      const bool c7_header_valid = !msg->fd && (msg->data[0] == 7U) && (msg->data[1] == 0xC7U) &&
                                    (msg->data[2] == 0xC7U) && (msg->data[6] == 0U) && (msg->data[7] == 0U);
      const bool host_admin_valid = toyota_tss3_08a_host && !toyota_corolla_hf && !msg->fd &&
                                    (msg->data[0] == 7U) && (msg->data[1] == 0xC9U) &&
                                    (msg->data[2] == 0xA8U) && (msg->data[3] <= 1U) &&
                                    (msg->data[4] == 0U) && (msg->data[5] == 0U) &&
                                    (msg->data[6] == 0U) && (msg->data[7] == 0U);
      if (host_admin_valid) {
        const bool release = msg->data[3] == 0U;
        if (release) {
          tx = true;
          toyota_tss3_08a_replacement_active = false;
          toyota_tss3_08a_first_host_frame = false;
        } else {
          // This is only a relay ownership handoff. It carries no steering
          // permission or target generation: controls_allowed remains the
          // ordinary openpilot actuation boundary for modified ID11 frames.
          const uint32_t native_age = safety_get_ts_elapsed(microsecond_timer_get(), toyota_tss3_08a_native_last_rx_ts);
          tx = (toyota_tss3_08a_native_history > 0U) && (native_age <= TOYOTA_TSS3_08A_REPLACEMENT_TIMEOUT_US);
          if (tx) {
            toyota_tss3_08a_replacement_active = true;
            toyota_tss3_08a_first_host_frame = true;
            toyota_tss3_08a_last_tx_ts = microsecond_timer_get();
            // The latest source generation may be cloned immediately as the
            // atomic handoff witness. Older pre-arm generations are ineligible.
            // If a newer native generation arrives first, rx_hook retires this
            // pre-arm candidate and makes only that newer generation eligible.
            for (uint8_t history_index = 0U; history_index < toyota_tss3_08a_native_history; history_index++) {
              toyota_tss3_08a_native_consumed[history_index] = history_index != 0U;
            }
          }
        }
      } else if (toyota_tss3_08a_host && !toyota_corolla_hf) {
        // F33 request-plane mode must not simultaneously command the old B6
        // resident-signer sideband.
        tx = false;
      } else {
        int target_angle = (msg->data[4] << 8U) | msg->data[5];
        target_angle = to_signed(target_angle, 16);
        const bool steer_control_enabled = msg->data[3] != 0U;
        if (toyota_corolla_hf) {
          tx = c7_header_valid &&
               !safety_max_limit_check(target_angle, TOYOTA_TSS3_COROLLA_ANGLE_STEERING_LIMITS.max_angle,
                                       -TOYOTA_TSS3_COROLLA_ANGLE_STEERING_LIMITS.max_angle) &&
               !steer_angle_cmd_checks(target_angle, steer_control_enabled, TOYOTA_TSS3_COROLLA_ANGLE_STEERING_LIMITS);
        } else {
          tx = c7_header_valid &&
               !toyota_f33_angle_cmd_checks(target_angle, steer_control_enabled,
                                            TOYOTA_F33_ANGLE_STEERING_LIMITS, TOYOTA_F33_ANGLE_STEERING_PARAMS);
        }
      }
    }
    if (oracle_transport) {
      const uint8_t header = msg->data[0];
      const uint8_t fragment = header >> 5U;
      const uint8_t seq = header & 0x1FU;
      const bool base_valid = !msg->fd && (GET_LEN(msg) == 8U) && (seq != 0U) && (fragment <= 4U);
      tx = false;

      if (base_valid && (fragment == 0U)) {
        // Fragment zero always restarts the bounded transaction. A later source
        // generation therefore recovers naturally from a partial host batch.
        toyota_tss3_08a_oracle_seq = seq;
        toyota_tss3_08a_oracle_next_fragment = 1U;
        tx = true;
      } else if (base_valid && (seq == toyota_tss3_08a_oracle_seq) &&
                 (fragment == toyota_tss3_08a_oracle_next_fragment)) {
        if (fragment == 4U) {
          const uint8_t inverted_seq = (uint8_t)(seq ^ 0xFFU);
          tx = (msg->data[3] == 0xC9U) && (msg->data[4] == 0xA8U) &&
               (msg->data[5] == inverted_seq) &&
               (msg->data[6] == 0x5AU) && (msg->data[7] == 0xA5U);
          toyota_tss3_08a_oracle_next_fragment = 0U;
        } else {
          toyota_tss3_08a_oracle_next_fragment++;
          tx = true;
        }
      } else {
        toyota_tss3_08a_oracle_next_fragment = 0U;
      }
    }
    if (host_08a) {
      tx = false;
      int matched_index = -1;
      int safety_drop_index = -1;

      if (toyota_tss3_08a_replacement_active && (toyota_tss3_08a_native_history > 0U) &&
          msg->fd && (GET_LEN(msg) == 32U)) {
        int oldest_unconsumed_index = -1;
        for (uint8_t history_index = 0U; history_index < toyota_tss3_08a_native_history; history_index++) {
          if (!toyota_tss3_08a_native_consumed[history_index]) {
            oldest_unconsumed_index = history_index;
          }
        }

        if (oldest_unconsumed_index >= 0) {
          const uint8_t history_index = (uint8_t)oldest_unconsumed_index;
          if (toyota_tss3_08a_first_host_frame) {
            // Exactly one source clone is used to witness the relay handoff.
            // Byte equality has no authority meaning after this frame.
            bool handoff_clone = true;
            for (uint8_t i = 0U; i < 32U; i++) {
              handoff_clone &= msg->data[i] == toyota_tss3_08a_native_frames[history_index][i];
            }
            if (handoff_clone) {
              matched_index = history_index;
              tx = true;
              desired_angle_last = SAFETY_CLAMP(angle_meas.values[0],
                                                -TOYOTA_F33_08A_ANGLE_STEERING_LIMITS.max_angle,
                                                 TOYOTA_F33_08A_ANGLE_STEERING_LIMITS.max_angle);
            }
          } else {
            // Match the exact source generation and permit only the owned
            // lateral fields and/or normal-DRCC identity, hold and bounds.
            // Every other Toyota request tuple remains source-exact.
            bool application_shape = true;
            bool actuation_valid = true;
            bool lateral_replacement = false;
            bool longitudinal_replacement = false;
            for (uint8_t i = 0U; i < 32U; i++) {
              const bool equal = msg->data[i] == toyota_tss3_08a_native_frames[history_index][i];
              const bool lateral_angle_byte = (i == 18U) || (i == 19U);
              const bool lateral_id_byte = i == 21U;
              const bool assist_gain_byte = i == 24U;
              const bool delayed_hold_byte = i == 4U;
              const bool longitudinal_id_byte = (i == 6U) || (i == 7U);
              const bool accel_bound_byte = (i == 8U) || (i == 9U) || (i == 11U) || (i == 12U);
              const bool mac_bit_byte = i >= 28U;
              if (delayed_hold_byte) {
                application_shape &= ((msg->data[i] ^ toyota_tss3_08a_native_frames[history_index][i]) & 0xDFU) == 0U;
              } else if (!lateral_angle_byte && !lateral_id_byte && !assist_gain_byte &&
                  !longitudinal_id_byte && !accel_bound_byte && !mac_bit_byte) {
                application_shape &= equal;
              }
              lateral_replacement |= !equal && (lateral_angle_byte || lateral_id_byte || assist_gain_byte);
              longitudinal_replacement |= !equal && (delayed_hold_byte || longitudinal_id_byte || accel_bound_byte);
            }

            // FV4 is source-generation identity; the MAC bits themselves may differ.
            application_shape &= (msg->data[28] & 0xF0U) ==
                                 (toyota_tss3_08a_native_frames[history_index][28] & 0xF0U);

            if (application_shape) {
              const uint8_t native_id = toyota_tss3_08a_native_frames[history_index][21] & 0x3FU;
              const uint8_t host_id = msg->data[21] & 0x3FU;
              const bool native_lateral_owner = (native_id == 0U) || (native_id == 4U) ||
                                                (native_id == 11U) || (native_id == 18U);
              const bool lateral_selector_shape = (msg->data[21] & 0xC0U) ==
                                                  (toyota_tss3_08a_native_frames[history_index][21] & 0xC0U);
              const bool host_lateral_active = (host_id == 11U) && (msg->data[24] == 100U);
              const bool host_lateral_inactive = (host_id == 0U) &&
                                                 (msg->data[24] == toyota_tss3_08a_native_frames[history_index][24]);
              int target_angle = (msg->data[18] << 8U) | msg->data[19];
              target_angle = to_signed(target_angle, 16);
              if (native_lateral_owner && lateral_selector_shape && host_lateral_active) {
                actuation_valid &= (controls_allowed || controls_allowed_lateral) &&
                                   !toyota_f33_angle_cmd_checks(target_angle, true, TOYOTA_F33_08A_ANGLE_STEERING_LIMITS,
                                                                TOYOTA_F33_ANGLE_STEERING_PARAMS);
              } else if (native_lateral_owner && lateral_selector_shape && host_lateral_inactive) {
                actuation_valid &= !toyota_f33_angle_cmd_checks(target_angle, false, TOYOTA_F33_08A_ANGLE_STEERING_LIMITS,
                                                                 TOYOTA_F33_ANGLE_STEERING_PARAMS);
              } else if (!native_lateral_owner) {
                // Unknown Toyota intervention IDs remain source-exact. Known
                // ordinary owners must always use openpilot's active or
                // inactive lateral command shape while replacement is active.
                application_shape &= !lateral_replacement;
              } else {
                application_shape = false;
              }
            }

            if (application_shape && longitudinal_replacement) {
              const bool source_no_request = (toyota_tss3_08a_native_frames[history_index][6] == 0x00U) &&
                                             (toyota_tss3_08a_native_frames[history_index][7] == 0x12U);
              const bool source_driver_override = (toyota_tss3_08a_native_frames[history_index][6] == 0x2CU) &&
                                                  (toyota_tss3_08a_native_frames[history_index][7] == 0x46U);
              const bool source_ordinary_drcc = (toyota_tss3_08a_native_frames[history_index][6] == 0x2DU) &&
                                                (toyota_tss3_08a_native_frames[history_index][7] == 0x47U);
              const bool source_delayed_hold = (toyota_tss3_08a_native_frames[history_index][6] == 0x2DU) &&
                                               (toyota_tss3_08a_native_frames[history_index][7] == 0x67U);
              const bool host_ordinary_drcc = (msg->data[6] == 0x2DU) && (msg->data[7] == 0x47U);
              const bool host_delayed_hold_clear = (msg->data[4] & 0x20U) == 0U;
              int accel_a = (msg->data[8] << 8U) | msg->data[9];
              int accel_b = (msg->data[11] << 8U) | msg->data[12];
              accel_a = to_signed(accel_a, 16);
              accel_b = to_signed(accel_b, 16);
              const bool longitudinal_shape = !toyota_stock_longitudinal &&
                                              (source_no_request || source_driver_override ||
                                               source_ordinary_drcc || source_delayed_hold) &&
                                              host_ordinary_drcc && host_delayed_hold_clear && (accel_a == accel_b);
              application_shape &= longitudinal_shape;
              if (longitudinal_shape) {
                actuation_valid &= !longitudinal_accel_checks(accel_a, TOYOTA_F33_LONG_LIMITS);
              }
            }

            tx = application_shape && actuation_valid;
            if (tx) {
              matched_index = history_index;
            } else if (application_shape && !actuation_valid) {
              safety_drop_index = history_index;
            }
          }
        }
      }

      if (tx) {
        toyota_tss3_08a_first_host_frame = false;
        toyota_tss3_08a_native_consumed[matched_index] = true;
        toyota_tss3_08a_last_tx_ts = microsecond_timer_get();
      } else if (safety_drop_index >= 0) {
        // Match normal Panda behavior: drop one structurally valid actuation
        // violation without changing request-plane ownership. Angle safety has
        // already reset its baseline to measured steering, so the next normal
        // CarController command can recover without a separate state machine.
        toyota_tss3_08a_native_consumed[safety_drop_index] = true;
      } else if (toyota_tss3_08a_replacement_active) {
        // Any malformed/replayed/mistimed host frame restores stock forwarding.
        toyota_tss3_08a_replacement_active = false;
        toyota_tss3_08a_first_host_frame = false;
      }
    }
    if (corolla_brake_cancel || camry_brake_cancel) {
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
    return tx;
  }

  // Check if msg is sent on BUS 0
  if (msg->bus == 0U) {
    // ACCEL: safety check on byte 1-2
    if (msg->addr == 0x343U) {
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

    if (msg->addr == 0x183U) {
      int desired_accel = (msg->data[0] << 8) | msg->data[1];
      desired_accel = to_signed(desired_accel, 16);

      tx = !longitudinal_accel_checks(desired_accel, TOYOTA_LONG_LIMITS);
    }

    // AEB: block all actuation. only used when DSU is unplugged
    if (msg->addr == 0x283U) {
      // only allow the checksum, which is the last byte
      bool block = (GET_BYTES(msg, 0, 4) != 0U) || (msg->data[4] != 0U) || (msg->data[5] != 0U);
      if (block) {
        tx = false;
      }
    }

    // STEERING_LTA angle steering check
    if (msg->addr == 0x191U) {
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
    if (toyota_secoc && (msg->addr == 0x131U)) {
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
    if (msg->addr == 0x2E4U) {
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

    // GAS PEDAL: safety check
    if (msg->addr == 0x200U) {
      if (longitudinal_interceptor_checks(msg)) {
        tx = false;
      }
    }
  }

  // UDS: Only tester present ("\x0F\x02\x3E\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (msg->addr == 0x750U) {
    // this address is sub-addressed. only allow tester present to radar (0xF)
    bool invalid_uds_msg = (GET_BYTES(msg, 0, 4) != 0x003E020FU) || (GET_BYTES(msg, 4, 4) != 0x0U);
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
      toyota_tss3_08a_first_host_frame = false;
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

  static const CanMsg TOYOTA_INTERCEPTOR_TX_MSGS[] = {
    TOYOTA_COMMON_LONG_TX_MSGS
    {0x200, 0, 6, .check_relay = false},  // gas interceptor
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

  const uint16_t TOYOTA_PARAM_SP_UNSUPPORTED_DSU = 1;
  const uint16_t TOYTOA_PARAM_SP_GAS_INTERCEPTOR = 2;

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
  toyota_tss3_08a_replacement_active = false;
  toyota_tss3_08a_native_history = 0U;
  toyota_tss3_08a_native_last_rx_ts = 0U;
  for (uint8_t i = 0U; i < TOYOTA_TSS3_08A_NATIVE_HISTORY_LEN; i++) {
    toyota_tss3_08a_native_consumed[i] = false;
  }
  toyota_tss3_08a_oracle_seq = 0U;
  toyota_tss3_08a_oracle_next_fragment = 0U;
  toyota_tss3_08a_first_host_frame = false;
  toyota_dbc_eps_torque_factor = param & TOYOTA_EPS_FACTOR;

  const bool toyota_unsupported_dsu = GET_FLAG(current_safety_param_sp, TOYOTA_PARAM_SP_UNSUPPORTED_DSU);
  enable_gas_interceptor = GET_FLAG(current_safety_param_sp, TOYTOA_PARAM_SP_GAS_INTERCEPTOR);

  // gas interceptor should not be used if openpilot is not controlling longitudinal or is a TSK car
  if (toyota_stock_longitudinal || toyota_secoc) {
    enable_gas_interceptor = false;
  }

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
        {0x1FDC0002, 0, 8, .check_relay = false},
        {0x08A, 0, 32, .check_relay = false},
        {0x101, 2, 8, .check_relay = false},
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
        {.msg = {{0x025, 0, 32, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x0AA, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
        {.msg = {{0x116, 0, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x101, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        // Relay-open F33 has authoritative native 0x08A only on source bus2.
        // Its downstream bus0 copy is Panda forwarding/TX echo, not an
        // independent RX source and must not be required for safety validity.
        {.msg = {{0x08A, 2, 32, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
        {.msg = {{0x00F, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
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
      TOYOTA_PCM_CRUISE_2_ADDR_CHECK
    };

    SET_RX_CHECKS(toyota_secoc_rx_checks, ret);
  } else if (toyota_lta) {
    // Check the quality flag for angle measurement when using LTA, since it's not set on TSS-P cars
    static RxCheck toyota_lta_rx_checks[] = {
      TOYOTA_RX_CHECKS(true)
      TOYOTA_PCM_CRUISE_2_ADDR_CHECK
    };

    SET_RX_CHECKS(toyota_lta_rx_checks, ret);
  } else {
    static RxCheck toyota_lka_rx_checks[] = {
      TOYOTA_RX_CHECKS(false)
      TOYOTA_PCM_CRUISE_2_ADDR_CHECK
    };
    static RxCheck toyota_lka_alt_brake_rx_checks[] = {
      TOYOTA_ALT_BRAKE_RX_CHECKS(false)
      TOYOTA_PCM_CRUISE_2_ADDR_CHECK
    };
    static RxCheck toyota_lka_unsupported_dsu_rx_checks[] = {
      TOYOTA_RX_CHECKS(false)
      TOYOTA_DSU_CRUISE_ADDR_CHECK
    };
    static RxCheck toyota_lka_alt_brake_unsupported_dsu_rx_checks[] = {
      TOYOTA_ALT_BRAKE_RX_CHECKS(false)
      TOYOTA_DSU_CRUISE_ADDR_CHECK
    };

    if (!toyota_alt_brake) {
      if (toyota_unsupported_dsu) {
        SET_RX_CHECKS(toyota_lka_unsupported_dsu_rx_checks, ret);
      } else {
        SET_RX_CHECKS(toyota_lka_rx_checks, ret);
      }
    } else {
      if (toyota_unsupported_dsu) {
        SET_RX_CHECKS(toyota_lka_alt_brake_unsupported_dsu_rx_checks, ret);
      } else {
        SET_RX_CHECKS(toyota_lka_alt_brake_rx_checks, ret);
      }
    }
  }

  if (enable_gas_interceptor) {
    SET_TX_MSGS(TOYOTA_INTERCEPTOR_TX_MSGS, ret);

    if (toyota_lta) {
      static RxCheck toyota_lta_interceptor_rx_checks[] = {
        TOYOTA_RX_CHECKS(true)
        TOYOTA_PCM_CRUISE_2_ADDR_CHECK
        TOYOTA_GAS_INTERCEPTOR_ADDR_CHECK
      };

      SET_RX_CHECKS(toyota_lta_interceptor_rx_checks, ret);
    } else {
      static RxCheck toyota_lka_interceptor_rx_checks[] = {
        TOYOTA_RX_CHECKS(false)
        TOYOTA_PCM_CRUISE_2_ADDR_CHECK
        TOYOTA_GAS_INTERCEPTOR_ADDR_CHECK
      };
      static RxCheck toyota_lka_alt_brake_interceptor_rx_checks[] = {
        TOYOTA_ALT_BRAKE_RX_CHECKS(false)
        TOYOTA_PCM_CRUISE_2_ADDR_CHECK
        TOYOTA_GAS_INTERCEPTOR_ADDR_CHECK
      };
      static RxCheck toyota_lka_unsupported_dsu_interceptor_rx_checks[] = {
        TOYOTA_RX_CHECKS(false)
        TOYOTA_DSU_CRUISE_ADDR_CHECK
        TOYOTA_GAS_INTERCEPTOR_ADDR_CHECK
      };
      static RxCheck toyota_lka_alt_brake_unsupported_dsu_interceptor_rx_checks[] = {
        TOYOTA_ALT_BRAKE_RX_CHECKS(false)
        TOYOTA_DSU_CRUISE_ADDR_CHECK
        TOYOTA_GAS_INTERCEPTOR_ADDR_CHECK
      };

      if (!toyota_alt_brake) {
        if (toyota_unsupported_dsu) {
          SET_RX_CHECKS(toyota_lka_unsupported_dsu_interceptor_rx_checks, ret);
        } else {
          SET_RX_CHECKS(toyota_lka_interceptor_rx_checks, ret);
        }
      } else {
        if (toyota_unsupported_dsu) {
          SET_RX_CHECKS(toyota_lka_alt_brake_unsupported_dsu_interceptor_rx_checks, ret);
        } else {
          SET_RX_CHECKS(toyota_lka_alt_brake_interceptor_rx_checks, ret);
        }
      }
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
