#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

// FIX-4: 需要向 MCU 上报真实的联网状态，必须引入对应组件头文件。
//        Arduino 原版在 send_status10() 里写 f[4]=WiFi.status()==WL_CONNECTED，
//        ESPHome 版却把 f[4]/f[5] 硬编码成 0 —— MCU 因此永远认为"秤未联网"，
//        表现为 LED 持续闪烁 + 未联网即进入休眠。
#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif
#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif

namespace esphome {
namespace s7_mcu {

static constexpr uint8_t  UART_MAX_FRAME = 40;
static constexpr uint32_t MCU_TIMEOUT_MS = 5000;

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

    // MCU timeout
    if (mcu_alive_ && millis() - last_frame_ms_ > MCU_TIMEOUT_MS) {
      mcu_alive_ = false;
      if (mcu_alive_sensor_ != nullptr)
        mcu_alive_sensor_->publish_state(false);
    }
  }

  float get_weight_kg() const { return weight_kg_; }
  uint16_t get_weight_raw() const { return weight_raw_; }
  bool is_mcu_alive() const { return mcu_alive_; }

  void set_weight_hi(uint8_t v) { weight_hi_ = v; }
  void set_weight_lo(uint8_t v) { weight_lo_ = v; }
  void set_weight_div(float v) { weight_div_ = v; }
  void set_weight_offset(float v) { weight_offset_ = v; }
  void set_weight_sensor(sensor::Sensor *s) { weight_sensor_ = s; }
  void set_mcu_alive_sensor(binary_sensor::BinarySensor *s) { mcu_alive_sensor_ = s; }
  void set_cs_ok(bool ok) { cs_ok_ = ok; }
  bool is_cs_ok() const { return cs_ok_; }
  void set_cs_ready_sensor(binary_sensor::BinarySensor *s) { cs_ready_sensor_ = s; }

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

  // 上报给 MCU 的状态帧：f[3]=CS1258就绪 f[4]=联网 f[5]=云端/HA在线
  void send_status10() {
    uint8_t f[7] = {0xC6, 0x04, 0x10, 0, 0, 0, 0};

    // f[3]: CS1258 阻抗前端是否就绪（由 cs1258 组件通过 set_cs_ok 写入）
    f[3] = cs_ok_ ? 1 : 0;

    // f[4]: WiFi 是否已连接 —— MCU 用它决定是否进入休眠。
    //       原实现恒为 0 => MCU 认为永远离线 => 未联网即关机。
    bool net_ok = false;
#ifdef USE_WIFI
    net_ok = wifi::global_wifi_component != nullptr && wifi::global_wifi_component->is_connected();
#endif
    f[4] = net_ok ? 1 : 0;

    // f[5]: 上位机（HA / MQTT）在线状态
    bool cloud_ok = false;
#ifdef USE_API
    cloud_ok = api::global_api_server != nullptr && api::global_api_server->is_connected();
#endif
#ifdef USE_MQTT
    if (!cloud_ok && mqtt::global_mqtt_client != nullptr)
      cloud_ok = mqtt::global_mqtt_client->is_connected();
#endif
    f[5] = cloud_ok ? 1 : 0;

    f[6] = xor_check(f, 6);
    mcu_send(f, 7);
  }

  void handle_cmd10(const uint8_t *frame, uint8_t total) {
    uint8_t pay_len = frame[1] > 1 ? frame[1] - 1 : 0;
    if (weight_hi_ < pay_len && weight_lo_ < pay_len) {
      weight_raw_ = ((uint16_t)frame[3 + weight_hi_] << 8)
                  | (uint16_t)frame[3 + weight_lo_];
      weight_kg_ = (float)weight_raw_ / weight_div_ + weight_offset_;
      if (weight_kg_ < 0) weight_kg_ = 0;

      if (weight_sensor_ != nullptr)
        weight_sensor_->publish_state(weight_kg_);
    }
    send_status10();
  }

  void handle_generic_c5(uint8_t cmd) {
    uint8_t f[5] = {0xC6, 0x02, cmd, 0x00, 0};
    f[4] = xor_check(f, 4);
    mcu_send(f, 5);
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

  // CS1258 就绪状态（由外部 cs1258 组件注入）
  bool cs_ok_{false};

  // Config
  uint8_t weight_hi_{0};
  uint8_t weight_lo_{1};
  float weight_div_{100.0f};
  float weight_offset_{0.0f};

  // Sensors
  sensor::Sensor *weight_sensor_{nullptr};
  binary_sensor::BinarySensor *mcu_alive_sensor_{nullptr};
  binary_sensor::BinarySensor *cs_ready_sensor_{nullptr};
};

}  // namespace s7_mcu
}  // namespace esphome
