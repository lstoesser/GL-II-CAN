#include "gl-ii-CAN.h"
#include <math.h>
#include <string.h>

namespace glii {
namespace {
bool validId(uint32_t id, bool extended) { return id <= (extended ? 0x1fffffffUL : 0x7ffUL); }
bool valid(const CanMsg& msg) { return msg.length <= 8 && validId(msg.id, msg.extended); }
}
GLIICAN::GLIICAN(const Config& config) : config_(config) {}
bool GLIICAN::begin() {
  if (started_) return true;
  if (!validId(config_.motorId, config_.extended) ||
      !validId(config_.positionVelocityId, config_.extended) ||
      !validId(config_.velocityId, config_.extended) ||
      !validId(config_.feedbackId, config_.feedbackExtended) ||
      !isfinite(config_.positionRange) || config_.positionRange <= 0 ||
      !isfinite(config_.velocityRange) || config_.velocityRange <= 0 ||
      !isfinite(config_.torqueRange) || config_.torqueRange <= 0) {
    lastError_ = ESP_ERR_INVALID_ARG; return false;
  }
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  switch (config_.bitrate) {
    case 1000000: timing = TWAI_TIMING_CONFIG_1MBITS(); break;
    case 500000: break;
    case 250000: timing = TWAI_TIMING_CONFIG_250KBITS(); break;
    case 125000: timing = TWAI_TIMING_CONFIG_125KBITS(); break;
    default: lastError_ = ESP_ERR_INVALID_ARG; return false;
  }
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(config_.txPin, config_.rxPin, TWAI_MODE_NORMAL);
  general.rx_queue_len = 32;
  general.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR |
                           TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_QUEUE_FULL;
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  lastError_ = twai_driver_install(&general, &timing, &filter);
  if (lastError_ != ESP_OK) return false;
  lastError_ = twai_start();
  if (lastError_ != ESP_OK) { twai_driver_uninstall(); return false; }
  started_ = true; return true;
}
bool GLIICAN::sendFrame(const CanMsg& msg) {
  if (!started_) { lastError_ = ESP_ERR_INVALID_STATE; return false; }
  if (!valid(msg)) { lastError_ = ESP_ERR_INVALID_ARG; return false; }
  twai_message_t tx = {};
  tx.identifier = msg.id; tx.extd = msg.extended; tx.rtr = msg.remote;
  tx.ss = config_.singleShot; tx.data_length_code = msg.length;
  if (!msg.remote) memcpy(tx.data, msg.data, msg.length);
  lastError_ = twai_transmit(&tx, 0);
  return lastError_ == ESP_OK;
}
bool GLIICAN::sendMsg(uint32_t id, uint8_t length, const uint8_t* data) {
  if (!data || length > 8) return false;
  CanMsg msg; msg.id = id; msg.extended = config_.extended; msg.length = length;
  memcpy(msg.data, data, length);
  return sendFrame(msg);
}
void GLIICAN::onFeedback(FeedbackCallback cb, void* data) { feedbackCallback_ = cb; feedbackContext_ = data; }
BusEvents GLIICAN::readBusEvents() {
  BusEvents events;
  uint32_t alerts = 0;
  if (started_ && twai_read_alerts(&alerts, 0) == ESP_OK) {
    events.busOff = alerts & TWAI_ALERT_BUS_OFF;
    events.txFailed = alerts & TWAI_ALERT_TX_FAILED;
    events.rxOverflow = alerts & TWAI_ALERT_RX_QUEUE_FULL;
  }
  return events;
}
bool GLIICAN::discardPendingCommands() {
  if (!started_) { lastError_ = ESP_ERR_INVALID_STATE; return false; }
  lastError_ = twai_clear_transmit_queue();
  return lastError_ == ESP_OK;
}
void GLIICAN::onFrame(FrameCallback cb, void* data) { frameCallback_ = cb; frameContext_ = data; }
void GLIICAN::update(int maxEvents) {
  expire();
  if (!started_) return;
  twai_message_t rx;
  while (maxEvents-- > 0 && twai_receive(&rx, 0) == ESP_OK) {
    if (rx.data_length_code > 8) continue;
    CanMsg msg; msg.id = rx.identifier; msg.extended = rx.extd; msg.remote = rx.rtr;
    msg.length = rx.data_length_code;
    if (!msg.remote) memcpy(msg.data, rx.data, msg.length);
    onReceive(msg);
  }
  expire();
}

// ID is of int8 type, taking the lower 8 bits of CAN_ID;
// ERR is of int8 type, with corresponding codes as follows:
// 0 — Disable;
// 1 — Enable;
// 8 — Over-voltage;
// 9 — Under-voltage;
// A — Over-current;
// B — MOS over-temperature;
// C — Motor winding over-temperature;
// D — Communication loss;
// E — Overload. The range -12.5 to 12.5 represents -12.5 to 12.5 rad;
// The range -200 to 200 represents -200 to 200 r/s;
// The range -10 to 10 represents -10 to 10 N-m;
// The drive temperature is of int8 type, with a range of -128 to 127°C;
// The motor temperature is of int8 type, with a range of -128 to 127°C.

void GLIICAN::onReceive(const CanMsg& msg) {
  if (!valid(msg)) return;
  expire();
  if (state_ == RequestState::Pending && !msg.remote && msg.id == replyId_ &&
      msg.extended == replyExtended_ && msg.length == replyLength_) {
    reply_ = msg; state_ = RequestState::Complete;
  }
  if (!msg.remote && msg.length == 8 && msg.id == config_.feedbackId &&
      msg.extended == config_.feedbackExtended && feedbackCallback_) {
    MotorFeedback f;
    f.id = uint8_t(msg.id); f.status = msg.data[0] >> 4;
    const uint16_t p = (uint16_t(msg.data[1]) << 8) | msg.data[2];
    const uint16_t v = (uint16_t(msg.data[3]) << 4) | (msg.data[4] >> 4);
    const uint16_t t = (uint16_t(msg.data[4] & 15) << 8) | msg.data[5];
    f.positionRad = decode(p, -config_.positionRange, config_.positionRange, 16);
    f.velocity = decode(v, -config_.velocityRange, config_.velocityRange, 12);
    f.torqueNm = decode(t, -config_.torqueRange, config_.torqueRange, 12);
    f.driverTemperatureC = msg.data[6] < 128 ? msg.data[6] : int(msg.data[6]) - 256;
    f.motorTemperatureC = msg.data[7] < 128 ? msg.data[7] : int(msg.data[7]) - 256;
    feedbackCallback_(f, feedbackContext_);
  }
  if (frameCallback_) frameCallback_(msg, frameContext_);
}

// UNIVERSAL CAN COMMANDS
// Enter Motor Control Mode 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0XFC
// Exit Motor Control Mode 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD
// Set Motor Current Position to Zero 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFE
// Clear Errors 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFB
bool GLIICAN::enterMotorControlMode() {
  const uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  return sendMsg(config_.motorId, sizeof(data), data);
}

bool GLIICAN::exitMotorControlMode() {
  const uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  return sendMsg(config_.motorId, sizeof(data), data);
}

bool GLIICAN::setCurrentPositionToZero() {
  const uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  return sendMsg(config_.motorId, sizeof(data), data);
}

bool GLIICAN::clearErrors() {
  const uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
  return sendMsg(config_.motorId, sizeof(data), data);
}

//When sending packets, all numerical values must be converted to integer numbers using thefollowing function before being sent to the motor.
uint16_t GLIICAN::encode(float value, float lo, float hi, unsigned bits) {
  if (!isfinite(value) || !isfinite(lo) || !isfinite(hi) || hi <= lo || bits == 0 || bits > 16) return 0;
  value = fmaxf(lo, fminf(hi, value));
  return uint16_t((value - lo) * ((1UL << bits) - 1) / (hi - lo));
}

// When receiving packets, the position, speed, and torque need to be converted to floating-point numbers using the following function
float GLIICAN::decode(uint16_t value, float lo, float hi, unsigned bits) {
  if (bits == 0 || bits > 16 || !isfinite(lo) || !isfinite(hi) || hi <= lo) return NAN;
  return value * (hi - lo) / ((1UL << bits) - 1) + lo;
}
bool GLIICAN::mit(float position, float velocity, float kp, float kd, float torque) {
  if (!isfinite(position) || !isfinite(velocity) || !isfinite(kp) || !isfinite(kd) || !isfinite(torque)) return false;
  uint16_t p = encode(position, -config_.positionRange, config_.positionRange, 16);
  uint16_t v = encode(velocity, -config_.velocityRange, config_.velocityRange, 12);
  uint16_t k = encode(kp, 0, 500, 12), d = encode(kd, 0, 5, 12);
  uint16_t t = encode(torque, -config_.torqueRange, config_.torqueRange, 12);
  uint8_t data[8] = {uint8_t(p >> 8), uint8_t(p), uint8_t(v >> 4), uint8_t((v << 4) | (k >> 8)),
                     uint8_t(k), uint8_t(d >> 4), uint8_t((d << 4) | (t >> 8)), uint8_t(t)};
  return sendMsg(config_.motorId, sizeof(data), data);
}
bool GLIICAN::position_velocity(float position, float velocity) {
  if (!isfinite(position) || !isfinite(velocity)) return false;
  // Preserve user's float32 little-endian packet format (ESP32).
  static_assert(sizeof(float) == 4, "Requires float32");
  uint8_t data[8]; memcpy(data, &position, 4); memcpy(data + 4, &velocity, 4);
  return sendMsg(config_.positionVelocityId, sizeof(data), data);
}
bool GLIICAN::velocity(float velocity) {
  if (!isfinite(velocity)) return false;
  uint8_t data[4]; memcpy(data, &velocity, 4);
  return sendMsg(config_.velocityId, sizeof(data), data);
}
bool GLIICAN::beginRequest(const CanMsg& msg, uint32_t id, bool extended, uint8_t length, uint32_t timeout) {
  if (state_ == RequestState::Pending || !validId(id, extended) || length > 8 || timeout == 0) return false;
  replyId_ = id; replyExtended_ = extended; replyLength_ = length;
  startedMs_ = millis(); timeoutMs_ = timeout; state_ = RequestState::Pending;
  if (!sendFrame(msg)) { state_ = RequestState::SendFailed; return false; }
  return true;
}
void GLIICAN::expire() {
  if (state_ == RequestState::Pending && uint32_t(millis() - startedMs_) >= timeoutMs_) state_ = RequestState::Timeout;
}
bool GLIICAN::takeReply(CanMsg& msg) {
  if (state_ != RequestState::Complete) return false;
  msg = reply_; state_ = RequestState::Idle; return true;
}
void GLIICAN::cancelRequest() { state_ = RequestState::Idle; }
const char* motorStatusName(uint8_t status) {
  switch (status) {
    case 0: return "Disabled"; case 1: return "Enabled";
    case 8: return "Over-voltage"; case 9: return "Under-voltage";
    case 10: return "Over-current"; case 11: return "MOS over-temperature";
    case 12: return "Motor over-temperature"; case 13: return "Communication loss";
    case 14: return "Overload"; default: return "Unknown";
  }
}
} // namespace glii
