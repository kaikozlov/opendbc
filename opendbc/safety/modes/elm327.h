#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/defaults.h"

// Local-only application command-5 experiment. The high bit enables exactly
// five classic input PDUs on one encoded bus while retaining normal ELM327
// diagnostic traffic. Production ELM327 behavior is unchanged when clear.
#define ELM327_PARAM_COMMAND5_PROBE 0x8000U
#define ELM327_PARAM_COMMAND5_BUS_MASK 0x0300U
#define ELM327_PARAM_COMMAND5_BUS_SHIFT 8U

static bool elm327_command5_probe = false;
static uint8_t elm327_command5_bus = 0U;

static safety_config elm327_init(uint16_t param) {
  elm327_command5_probe = GET_FLAG(param, ELM327_PARAM_COMMAND5_PROBE);
  elm327_command5_bus = (uint8_t)((param & ELM327_PARAM_COMMAND5_BUS_MASK) >> ELM327_PARAM_COMMAND5_BUS_SHIFT);
  return nooutput_init(param);
}

static bool elm327_tx_hook(const CANPacket_t *msg) {
  const unsigned int GM_CAMERA_DIAG_ADDR = 0x24BU;

  bool tx = true;
  int len = GET_LEN(msg);
  const bool command5_input = elm327_command5_probe && (msg->bus == elm327_command5_bus) &&
                              (msg->addr >= 0x01BU) && (msg->addr <= 0x01FU);

  // All ISO 15765-4 messages must be 8 bytes long
  if (len != 8) {
    tx = false;
  }

  // Check valid 29 bit send addresses for ISO 15765-4
  // Check valid 11 bit send addresses for ISO 15765-4
  if (!command5_input && (msg->addr != 0x18DB33F1U) && ((msg->addr & 0x1FFF00FFU) != 0x18DA00F1U) &&
      ((msg->addr & 0x1FFFFF00U) != 0x600U) && ((msg->addr & 0x1FFFFF00U) != 0x700U) &&
      (msg->addr != GM_CAMERA_DIAG_ADDR)) {
    tx = false;
  }

  // GM camera uses non-standard diagnostic address, this has no control message address collisions
  if ((msg->addr == GM_CAMERA_DIAG_ADDR) && (len == 8)) {
    // Only allow known frame types for ISO 15765-2
    if ((msg->data[0] & 0xF0U) > 0x30U) {
      tx = false;
    }
  }
  return tx;
}

// If safety_param == 0, bus 1 is multiplexed to the OBD-II port
const safety_hooks elm327_hooks = {
  .init = elm327_init,
  .rx = default_rx_hook,
  .tx = elm327_tx_hook,
};
