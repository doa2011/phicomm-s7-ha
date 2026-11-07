#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/cs1258/cs1258.h"

namespace esphome {
namespace s7_mcu {

static constexpr uint8_t  UART_MAX_FRAME = 40;
static constexpr uint32_t MCU_TIMEOUT_MS = 5000;
static constexpr float WEIGHT_ON_THRESH = 3.0f;
static constexpr float WEIGHT_STABLE_DELTA = 0.3f;
static constexpr uint32_t WEIGHT_STABLE_MS = 3000;
static constexpr uint32_t PROFILE_SYNC_WAIT_MS = 12000;
static constexpr uint32_t MEMBER_ARM_TIMEOUT_MS = 120000;
static constexpr uint32_t IMPEDANCE_COOL_MS = 20000;
static const char *const TAG = "s7_mcu";

class S7McuComponent : public Component, public uart::UARTDevice {
 public:
  void setup() override {
    memset(buf_, 0, sizeof(buf_));
    pos_ = 0;
    expected_ = 0;
    synced_ = false;
    mcu_alive_ = false;
    last_frame_ms_ = 0;
    weight_kg_ = 0;
    weight_raw_ = 0;
    person_on_scale_ = false;
    measurement_done_ = false;
    stable_weight_ = 0.0f;
    weight_stable_since_ = 0;
    last_impedance_ms_ = 0;
    scan_count_ = 0;
    last_published_weight_raw_ = 0xFFFF;
    if (mcu_alive_sensor_ != nullptr) mcu_alive_sensor_->publish_state(false);
    if (measurement_valid_sensor_ != nullptr) measurement_valid_sensor_->publish_state(false);
  }

  void loop() override {
    while (available()) {
      uint8_t b;
      read_byte(&b);

      if (!synced_) {
        if (b == 0xC5 || b == 0xC6) {
          buf_[0] = b;
          pos_ = 1;
          expected_ = 0;
          synced_ = true;
        }
        continue;
      }

      buf_[pos_++] = b;

      if (pos_ == 2) {
        expected_ = b + 3;
        if (expected_ > UART_MAX_FRAME || expected_ < 4) {
          synced_ = false;
          continue;
        }
      }

      if (pos_ >= expected_ && expected_ >= 4) {
        if (xor_check(buf_, expected_ - 1) == buf_[expected_ - 1]) {
          dispatch_frame(buf_, expected_);
        }
        synced_ = false;
      }

      if (pos_ >= UART_MAX_FRAME) {
        synced_ = false;
      }
    }

    update_weight_measurement_();

    // MCU timeout
    if (mcu_alive_ && millis() - last_frame_ms_ > MCU_TIMEOUT_MS) {
      mcu_alive_ = false;
      if (mcu_alive_sensor_ != nullptr)
        mcu_alive_sensor_->publish_state(false);
      // A silent MCU means the current weighing session has ended/reset.
      person_on_scale_ = false;
      measurement_done_ = false;
      weight_kg_ = 0.0f;
      weight_raw_ = 0;
      last_published_weight_raw_ = 0;
      if (weight_sensor_ != nullptr)
        weight_sensor_->publish_state(0.0f);
      if (weight_raw_sensor_ != nullptr)
        weight_raw_sensor_->publish_state(0.0f);
      if (measurement_valid_sensor_ != nullptr)
        measurement_valid_sensor_->publish_state(false);
    }
  }

  float get_weight_kg() const { return weight_kg_; }
  uint16_t get_weight_raw() const { return weight_raw_; }
  bool is_mcu_alive() const { return mcu_alive_; }
  uint32_t get_scan_count() const { return scan_count_; }
  bool set_active_member(uint8_t member) {
    if (person_on_scale_) return false;
    active_member_ = member <= 3 ? member : 0;
    active_member_synced_ = true;
    active_member_set_ms_ = millis();
    return true;
  }
  uint8_t get_session_member() const { return session_member_; }

  void set_weight_hi(uint8_t v) { weight_hi_ = v; }
  void set_weight_lo(uint8_t v) { weight_lo_ = v; }
  void set_weight_div(float v) { weight_div_ = v; }
  void set_weight_offset(float v) { weight_offset_ = v; }
  void set_weight_sensor(sensor::Sensor *s) { weight_sensor_ = s; }
  void set_weight_raw_sensor(sensor::Sensor *s) { weight_raw_sensor_ = s; }
  void set_measurement_id_sensor(sensor::Sensor *s) { measurement_id_sensor_ = s; }
  void set_measurement_valid_sensor(binary_sensor::BinarySensor *s) { measurement_valid_sensor_ = s; }
  void set_mcu_alive_sensor(binary_sensor::BinarySensor *s) { mcu_alive_sensor_ = s; }
  void set_cs1258_component(cs1258::CS1258Component *c) { cs1258_ = c; }

  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  static uint8_t xor_check(const uint8_t *d, uint8_t n) {
    uint8_t x = 0;
    for (uint8_t i = 0; i < n; i++) x ^= d[i];
    return x;
  }

  void mcu_send(const uint8_t *d, uint8_t len) {
    write_array(d, len);
  }

  // Exact Arduino status frame semantics: f[3]=m.csOk; f[4]/f[5] stay zero.
  void send_status10() {
    uint8_t f[7] = {0xC6, 0x04, 0x10, 0, 0, 0, 0};
    f[3] = (cs1258_ != nullptr && cs1258_->is_ok()) ? 1 : 0;
    f[6] = xor_check(f, 6);
    mcu_send(f, 7);
  }

  void handle_cmd10(const uint8_t *frame, uint8_t total) {
    uint8_t pay_len = frame[1] > 1 ? frame[1] - 1 : 0;
    if (weight_hi_ < pay_len && weight_lo_ < pay_len) {
      weight_raw_ = ((uint16_t)frame[3 + weight_hi_] << 8)
                  | (uint16_t)frame[3 + weight_lo_];
      if (weight_div_ <= 0.0f) {
        ESP_LOGE(TAG, "Invalid weight_div=%.3f; frame ignored", weight_div_);
        send_status10();
        return;
      }
      weight_kg_ = (float)weight_raw_ / weight_div_ + weight_offset_;
      if (weight_kg_ < 0) weight_kg_ = 0;

      // Keep replying to every MCU frame, but publish/log only when the raw
      // weight changes. This avoids flooding HA and the UART debug log.
      if (weight_raw_ != last_published_weight_raw_) {
        if (weight_sensor_ != nullptr)
          weight_sensor_->publish_state(weight_kg_);
        if (weight_raw_sensor_ != nullptr)
          weight_raw_sensor_->publish_state(weight_raw_);
        ESP_LOGD(TAG, "RX C5 10 raw=%u weight=%.2f kg", weight_raw_, weight_kg_);
        last_published_weight_raw_ = weight_raw_;
      }
    }
    send_status10();
  }

  void handle_generic_c5(uint8_t cmd) {
    uint8_t f[5] = {0xC6, 0x02, cmd, 0x00, 0};
    f[4] = xor_check(f, 4);
    mcu_send(f, 5);
    ESP_LOGD(TAG, "RX C5 cmd=%02X; generic response sent", cmd);
  }

  void dispatch_frame(const uint8_t *fr, uint8_t total) {
    bool was_alive = mcu_alive_;
    mcu_alive_ = true;
    last_frame_ms_ = millis();

    if (!was_alive && mcu_alive_sensor_ != nullptr)
      mcu_alive_sensor_->publish_state(true);

    if (fr[0] == 0xC5) {
      if (fr[2] == 0x10)
        handle_cmd10(fr, total);
      else
        handle_generic_c5(fr[2]);
    }
  }

  void update_weight_measurement_() {
    const uint32_t now = millis();

    // Expire an unused member selection even while the scale is empty.
    // Once a person steps on, person_on_scale_ locks the member until step-off.
    if (!person_on_scale_ && active_member_synced_ &&
        (uint32_t)(now - active_member_set_ms_) >= MEMBER_ARM_TIMEOUT_MS) {
      active_member_ = 0;
      active_member_synced_ = false;
      ESP_LOGW(TAG, "Prepared member selection expired; next measurement will be Unassigned unless re-armed");
    }

    if (weight_kg_ < WEIGHT_ON_THRESH) {
      if (person_on_scale_) {
        ESP_LOGI(TAG, "Weight session ended; member assignment cleared");
        person_on_scale_ = false;
        measurement_done_ = false;
        active_member_ = 0;
        active_member_synced_ = false;
        if (measurement_valid_sensor_ != nullptr)
          measurement_valid_sensor_->publish_state(false);
      }
      return;
    }
    if (!person_on_scale_) {
      person_on_scale_ = true;
      measurement_done_ = false;
      if (measurement_valid_sensor_ != nullptr)
        measurement_valid_sensor_->publish_state(false);
      if (cs1258_ != nullptr) cs1258_->clear_body_valid();
      stable_weight_ = weight_kg_;
      weight_stable_since_ = now;
      weight_session_started_ = now;
      ESP_LOGI(TAG, "Weight session started: %.2f kg", weight_kg_);
    } else {
      float diff = weight_kg_ - stable_weight_;
      if (diff < 0.0f) diff = -diff;
      if (diff > WEIGHT_STABLE_DELTA) {
        stable_weight_ = weight_kg_;
        weight_stable_since_ = now;
        ESP_LOGD(TAG, "Weight changed; stability timer restarted");
      }
    }
    const bool stable = (uint32_t)(now - weight_stable_since_) >= WEIGHT_STABLE_MS;
    const bool profile_ready = active_member_synced_ ||
        (uint32_t)(now - weight_session_started_) >= PROFILE_SYNC_WAIT_MS;
    const bool cooldown_done = (uint32_t)(now - last_impedance_ms_) > IMPEDANCE_COOL_MS;
    if (stable && profile_ready && !measurement_done_ && cooldown_done && cs1258_ != nullptr) {
      ESP_LOGI(TAG, "Weight stable; starting CS1258 scan");
      // A scale session gets exactly one attempt, including failed scans.
      // The next attempt is allowed only after the person steps off.
      measurement_done_ = true;
      session_member_ = active_member_synced_ ? active_member_ : 0;
      cs1258_->scan();
      last_impedance_ms_ = millis();
      ++scan_count_;
      const bool valid = cs1258_->is_body_valid();
      if (measurement_valid_sensor_ != nullptr)
        measurement_valid_sensor_->publish_state(valid);
      if (measurement_id_sensor_ != nullptr)
        measurement_id_sensor_->publish_state(static_cast<float>(scan_count_));
      ESP_LOGI(TAG, "Measurement #%u: scan_ok=%s body_valid=%s duration=%u ms",
               (unsigned)scan_count_, cs1258_->is_ok() ? "YES" : "NO",
               valid ? "YES" : "NO",
               (unsigned)cs1258_->get_scan_duration_ms());
      ESP_LOGD(TAG, "CS1258: %s", cs1258_->get_scan_log());
    }
  }

  // Frame parser state
  uint8_t buf_[UART_MAX_FRAME];
  uint8_t pos_{0};
  uint8_t expected_{0};
  bool synced_{false};

  // MCU state
  bool mcu_alive_{false};
  uint32_t last_frame_ms_{0};
  float weight_kg_{0};
  uint16_t weight_raw_{0};

  // Config
  uint8_t weight_hi_{0};
  uint8_t weight_lo_{1};
  float weight_div_{100.0f};
  float weight_offset_{0.0f};

  // Sensors
  sensor::Sensor *weight_sensor_{nullptr};
  sensor::Sensor *weight_raw_sensor_{nullptr};
  sensor::Sensor *measurement_id_sensor_{nullptr};
  binary_sensor::BinarySensor *measurement_valid_sensor_{nullptr};
  binary_sensor::BinarySensor *mcu_alive_sensor_{nullptr};
  cs1258::CS1258Component *cs1258_{nullptr};

  bool person_on_scale_{false};
  bool measurement_done_{false};
  float stable_weight_{0.0f};
  uint32_t weight_stable_since_{0};
  uint32_t weight_session_started_{0};
  uint32_t active_member_set_ms_{0};
  uint32_t last_impedance_ms_{0};
  uint32_t scan_count_{0};
  uint8_t active_member_{0};
  uint8_t session_member_{0};
  bool active_member_synced_{false};
  uint16_t last_published_weight_raw_{0xFFFF};
};

}  // namespace s7_mcu
}  // namespace esphome
