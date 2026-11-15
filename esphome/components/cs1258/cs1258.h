#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>

namespace esphome {
namespace cs1258 {

struct ChResult {
  uint32_t raw;
  uint16_t ohm;
};

class CS1258Component : public Component {
 public:
  void setup() override {
    pinMode(pin_cs_, OUTPUT);
    pinMode(pin_clk_, OUTPUT);
    pinMode(pin_dio_, INPUT);
    digitalWrite(pin_cs_, HIGH);
    digitalWrite(pin_clk_, HIGH);
  }

  float get_setup_priority() const override { return setup_priority::DATA; }

  /// Run a full impedance scan. Blocking (~2s). Call from script/lambda.
  void scan() {
    scan_ok_ = false;
    ready_ = false;
    body_valid_ = false;
    body_res_ = 0;
    z_eff_ohm_ = 0.0f;
    z1_ = z2_ = z3_ = z4_ = z5_ = 0;
    short_adc_ = cal_a_ = cal_b_ = 0;
    lr_leg_ = rl_leg_ = lr_hand_ = r_leg_ = l_leg_ = {};
    lh_ll_ = lh_rl_ = rh_ll_ = rh_rl_ = {};
    scan_log_[0] = '\0';
    if (impedance_sensor_ != nullptr) impedance_sensor_->publish_state(0.0f);
    if (body_res_sensor_ != nullptr) body_res_sensor_->publish_state(0);
    publish_channel_(lr_leg_sensor_, 0);
    publish_channel_(rl_leg_sensor_, 0);
    publish_channel_(lr_hand_sensor_, 0);
    publish_channel_(r_leg_sensor_, 0);
    publish_channel_(l_leg_sensor_, 0);
    publish_channel_(lh_ll_sensor_, 0);
    publish_channel_(lh_rl_sensor_, 0);
    publish_channel_(rh_ll_sensor_, 0);
    publish_channel_(rh_rl_sensor_, 0);
    publish_channel_(z1_sensor_, 0);
    publish_channel_(z2_sensor_, 0);
    publish_channel_(z3_sensor_, 0);
    publish_channel_(z4_sensor_, 0);
    publish_channel_(z5_sensor_, 0);
    publish_channel_(short_adc_sensor_, 0);
    publish_channel_(cal_a_sensor_, 0);
    publish_channel_(cal_b_sensor_, 0);
    publish_channel_(scan_ms_sensor_, 0);
    uint32_t t0 = millis();

    if (!reset_init()) {
      append_log_("reset_init=FAIL");
      scan_duration_ms_ = millis() - t0;
      publish_channel_(scan_ms_sensor_, scan_duration_ms_);
      return;
    }

    if (!pre_baseline()) {
      append_log_("pre_baseline=FAIL");
      scan_duration_ms_ = millis() - t0;
      publish_channel_(scan_ms_sensor_, scan_duration_ms_);
      return;
    }
    append_log_("init=OK pre=OK\n");

    // Baselines
    short_adc_ = read_baseline(0x33);
    cal_a_     = read_baseline(0x13);
    cal_b_     = read_baseline(0x23);
    append_log_("baseline short=%lu cal_a=%lu cal_b=%lu span=%ld\n",
                (unsigned long) short_adc_, (unsigned long) cal_a_,
                (unsigned long) cal_b_, (long)((int32_t)cal_b_ - (int32_t)cal_a_ - (int32_t)cal_offset_));

    // 9 channels
    lr_leg_  = read_channel(0xEB);
    rl_leg_  = read_channel(0xBE);
    lr_hand_ = read_channel(0x14);
    r_leg_   = read_channel(0xDB);
    l_leg_   = read_channel(0x8E);
    lh_ll_   = read_channel(0x3C);
    lh_rl_   = read_channel(0x28);
    rh_ll_   = read_channel(0x7D);
    rh_rl_   = read_channel(0x69);
    append_log_("EB=%u BE=%u 14=%u DB=%u 8E=%u 3C=%u 28=%u 7D=%u 69=%u\n",
                lr_leg_.ohm, rl_leg_.ohm, lr_hand_.ohm, r_leg_.ohm, l_leg_.ohm,
                lh_ll_.ohm, lh_rl_.ohm, rh_ll_.ohm, rh_rl_.ohm);

    // Composite impedances
    body_res_ = half2(rl_leg_.ohm, lr_leg_.ohm);
    z1_ = half_s((int32_t)lr_hand_.ohm + lh_rl_.ohm - rh_rl_.ohm);
    z2_ = half_s((int32_t)rh_rl_.ohm + lr_hand_.ohm - lh_rl_.ohm);
    z3_ = half_s((int32_t)rh_ll_.ohm - rh_rl_.ohm + body_res_);
    z4_ = half_s((int32_t)rh_rl_.ohm - rh_ll_.ohm + body_res_);
    z5_ = (uint16_t)(((uint32_t)rh_ll_.ohm + lh_rl_.ohm) / 50);

    body_valid_ = body_res_ >= 50 && body_res_ <= 3000;
    z_eff_ohm_ = body_valid_ ? body_res_ * z_scale_ + z_offset_ : 0;

    scan_ok_ = true;
    scan_duration_ms_ = millis() - t0;

    // Publish
    if (impedance_sensor_ != nullptr)
      impedance_sensor_->publish_state(z_eff_ohm_);
    if (body_res_sensor_ != nullptr)
      body_res_sensor_->publish_state(body_res_);
    publish_channel_(lr_leg_sensor_, lr_leg_.ohm);
    publish_channel_(rl_leg_sensor_, rl_leg_.ohm);
    publish_channel_(lr_hand_sensor_, lr_hand_.ohm);
    publish_channel_(r_leg_sensor_, r_leg_.ohm);
    publish_channel_(l_leg_sensor_, l_leg_.ohm);
    publish_channel_(lh_ll_sensor_, lh_ll_.ohm);
    publish_channel_(lh_rl_sensor_, lh_rl_.ohm);
    publish_channel_(rh_ll_sensor_, rh_ll_.ohm);
    publish_channel_(rh_rl_sensor_, rh_rl_.ohm);
    publish_channel_(z1_sensor_, z1_);
    publish_channel_(z2_sensor_, z2_);
    publish_channel_(z3_sensor_, z3_);
    publish_channel_(z4_sensor_, z4_);
    publish_channel_(z5_sensor_, z5_);
    publish_channel_(short_adc_sensor_, short_adc_);
    publish_channel_(cal_a_sensor_, cal_a_);
    publish_channel_(cal_b_sensor_, cal_b_);
    publish_channel_(scan_ms_sensor_, scan_duration_ms_);
    append_log_("body_res=%u z_eff=%.1f valid=%s duration=%lu ms", body_res_,
                z_eff_ohm_, body_valid_ ? "YES" : "NO",
                (unsigned long)scan_duration_ms_);
  }

  // --- Accessors ---
  bool is_ok() const { return scan_ok_; }
  /// True when the CS1258 reset and register initialization succeeded.
  bool is_ready() const { return ready_; }
  bool is_body_valid() const { return body_valid_; }
  void clear_body_valid() { body_valid_ = false; }
  uint32_t get_scan_duration_ms() const { return scan_duration_ms_; }
  float get_z_eff_ohm() const { return z_eff_ohm_; }
  uint16_t get_body_res() const { return body_res_; }
  uint16_t get_z1() const { return z1_; }
  uint16_t get_z2() const { return z2_; }
  uint16_t get_z3() const { return z3_; }
  uint16_t get_z4() const { return z4_; }
  uint16_t get_z5() const { return z5_; }
  uint32_t get_short_adc() const { return short_adc_; }
  uint32_t get_cal_a() const { return cal_a_; }
  uint32_t get_cal_b() const { return cal_b_; }
  const char *get_scan_log() const { return scan_log_; }

  // --- Config setters ---
  void set_pin_cs(uint8_t p) { pin_cs_ = p; }
  void set_pin_clk(uint8_t p) { pin_clk_ = p; }
  void set_pin_dio(uint8_t p) { pin_dio_ = p; }
  void set_z_scale(float v) { z_scale_ = v; }
  void set_z_offset(float v) { z_offset_ = v; }
  void set_cal_offset(uint16_t v) { cal_offset_ = v; }
  void set_impedance_sensor(sensor::Sensor *s) { impedance_sensor_ = s; }
  void set_body_res_sensor(sensor::Sensor *s) { body_res_sensor_ = s; }
  void set_lr_leg_sensor(sensor::Sensor *s) { lr_leg_sensor_ = s; }
  void set_rl_leg_sensor(sensor::Sensor *s) { rl_leg_sensor_ = s; }
  void set_lr_hand_sensor(sensor::Sensor *s) { lr_hand_sensor_ = s; }
  void set_r_leg_sensor(sensor::Sensor *s) { r_leg_sensor_ = s; }
  void set_l_leg_sensor(sensor::Sensor *s) { l_leg_sensor_ = s; }
  void set_lh_ll_sensor(sensor::Sensor *s) { lh_ll_sensor_ = s; }
  void set_lh_rl_sensor(sensor::Sensor *s) { lh_rl_sensor_ = s; }
  void set_rh_ll_sensor(sensor::Sensor *s) { rh_ll_sensor_ = s; }
  void set_rh_rl_sensor(sensor::Sensor *s) { rh_rl_sensor_ = s; }
  void set_z1_sensor(sensor::Sensor *s) { z1_sensor_ = s; }
  void set_z2_sensor(sensor::Sensor *s) { z2_sensor_ = s; }
  void set_z3_sensor(sensor::Sensor *s) { z3_sensor_ = s; }
  void set_z4_sensor(sensor::Sensor *s) { z4_sensor_ = s; }
  void set_z5_sensor(sensor::Sensor *s) { z5_sensor_ = s; }
  void set_short_adc_sensor(sensor::Sensor *s) { short_adc_sensor_ = s; }
  void set_cal_a_sensor(sensor::Sensor *s) { cal_a_sensor_ = s; }
  void set_cal_b_sensor(sensor::Sensor *s) { cal_b_sensor_ = s; }
  void set_scan_ms_sensor(sensor::Sensor *s) { scan_ms_sensor_ = s; }

 protected:
  void append_log_(const char *fmt, ...) {
    const size_t used = strlen(scan_log_);
    if (used >= sizeof(scan_log_) - 1) return;
    va_list args;
    va_start(args, fmt);
    vsnprintf(scan_log_ + used, sizeof(scan_log_) - used, fmt, args);
    va_end(args);
  }

  void publish_channel_(sensor::Sensor *sensor, uint32_t value) {
    if (sensor != nullptr) sensor->publish_state(static_cast<float>(value));
  }

  // --- Low-level bit-bang ---
  void cs_delay(uint16_t us) { delayMicroseconds(us); }

  void cs_begin() {
    digitalWrite(pin_clk_, LOW); cs_delay(800);
    digitalWrite(pin_cs_, LOW);  cs_delay(2);
  }

  void cs_end() {
    digitalWrite(pin_cs_, HIGH); cs_delay(2);
    digitalWrite(pin_clk_, HIGH);
    pinMode(pin_dio_, INPUT); delay(2);
  }

  void cs_write_byte(uint8_t v) {
    pinMode(pin_dio_, OUTPUT);
    for (uint8_t i = 0; i < 8; i++) {
      digitalWrite(pin_dio_, (v & 0x80) ? HIGH : LOW);
      cs_delay(1); digitalWrite(pin_clk_, HIGH);
      cs_delay(1); digitalWrite(pin_clk_, LOW);
      v <<= 1; cs_delay(1);
    }
    cs_delay(2);
  }

  uint8_t cs_read_byte() {
    uint8_t v = 0;
    pinMode(pin_dio_, INPUT);
    for (uint8_t i = 0; i < 8; i++) {
      digitalWrite(pin_clk_, HIGH); v <<= 1; cs_delay(1);
      digitalWrite(pin_clk_, LOW);
      if (digitalRead(pin_dio_)) v |= 1;
      cs_delay(1);
    }
    return v;
  }

  void cs_write_reg(uint8_t reg, uint8_t val) {
    cs_begin(); cs_write_byte(0x80 | reg); cs_delay(2);
    cs_write_byte(val); cs_end();
  }

  uint8_t cs_read_reg(uint8_t reg) {
    cs_begin(); cs_write_byte(reg);
    uint8_t v = cs_read_byte(); cs_end();
    return v;
  }

  bool cs_write_verify(uint8_t reg, uint8_t val, uint8_t retries = 2) {
    for (uint8_t i = 0; i < retries; i++) {
      cs_write_reg(reg, val); delay(1);
      if (cs_read_reg(reg) == val) return true;
      delay(1);
    }
    return false;
  }

  uint32_t cs_read_adc24() {
    cs_begin(); cs_write_byte(0x09); cs_delay(2);
    uint8_t b0 = cs_read_byte(), b1 = cs_read_byte(), b2 = cs_read_byte();
    cs_end();
    return ((uint32_t)(b0 ^ 0x80) << 16) | ((uint32_t)b1 << 8) | b2;
  }

  void cs_read_adc24_raw(uint8_t *b0, uint8_t *b1, uint8_t *b2) {
    cs_begin(); cs_write_byte(0x09); cs_delay(2);
    *b0 = cs_read_byte(); *b1 = cs_read_byte(); *b2 = cs_read_byte();
    cs_end();
  }

  uint32_t cs_sample_avg(uint8_t n, uint16_t jump) {
    uint32_t sum = 0, prev = 0;
    uint8_t acc = 0, kept = 0;
    for (uint8_t t = 0; t < 110 && acc < n; t++) {
      delay(3);
      uint32_t r = cs_read_adc24();
      if (acc > 0) {
        uint32_t d = r > prev ? r - prev : prev - r;
        if (d > jump) { prev = r; continue; }
      }
      prev = r; acc++;
      if (acc > 4) { sum += r; kept++; }
      yield();
    }
    return kept ? sum / kept : prev;
  }

  // --- Init / Baseline / Channel ---
  bool reset_init() {
    pinMode(pin_cs_, OUTPUT); pinMode(pin_clk_, OUTPUT); pinMode(pin_dio_, INPUT);
    digitalWrite(pin_cs_, HIGH); digitalWrite(pin_clk_, HIGH);
    delay(100);

    cs_write_reg(0xEA, 0x96);
    delay(10);
    uint8_t r0a = cs_read_reg(0x0A);
    if (!(r0a & 0x40)) return false;
    if (!cs_write_verify(0x59, 0x6A)) return false;

    uint8_t r13 = cs_read_reg(0x13);

    struct Step { uint8_t reg; uint8_t val; };
    Step steps[] = {
      {0x59, 0xA6}, {0x5A, 0x5A}, {0x12, r13}, {0x13, r13}, {0x10, 0xBF}
    };
    for (auto &s : steps) {
      if (!cs_write_verify(s.reg, s.val)) return false;
    }
    ready_ = true;
    return true;
  }

  bool pre_baseline() {
    if (!cs_write_verify(0x05, 0x42)) return false;
    cs_write_reg(0x00, 0xCA);
    delay(20);
    if (!cs_write_verify(0x02, 0x4E)) return false;
    return true;
  }

  uint32_t read_baseline(uint8_t reg8val) {
    cs_write_reg(0x08, reg8val);
    delay(2);
    cs_write_reg(0x07, 0x00);
    delay(2);
    cs_write_reg(0x00, 0xCF);
    delay(5);

    // Discard first two readings, preserving the original Arduino diagnostics.
    uint8_t b0, b1, b2;
    cs_read_adc24_raw(&b0, &b1, &b2);
    append_log_("bl_%02X first=%02X%02X%02X", reg8val, b0, b1, b2);
    delay(5);
    cs_read_adc24_raw(&b0, &b1, &b2);
    append_log_(" second=%02X%02X%02X", b0, b1, b2);

    uint32_t v = cs_sample_avg(8, 500);
    append_log_(" avg=%lu\n", (unsigned long)v);
    cs_write_reg(0x00, 0xCA);
    delay(2);
    return v;
  }

  uint16_t adc_to_ohm(uint32_t raw) {
    // Cast operands before subtraction to avoid unsigned underflow.
    int32_t span = (int32_t)cal_b_ - (int32_t)cal_a_ - (int32_t)cal_offset_;
    if (span > 100) {
      if (raw <= short_adc_) return 0;
      float z = (float)(raw - short_adc_) * 7000.0f / (float)span;
      if (z < 0) return 0;
      if (z > 65535) return 65535;
      return (uint16_t)z;
    }
    // Fallback
    static const uint32_t ADC_ZERO = 8388608;
    if (raw <= ADC_ZERO) return 0;
    uint32_t delta = raw - ADC_ZERO;
    uint32_t ohm = delta / 512;
    return ohm > 65535 ? 65535 : (uint16_t)ohm;
  }

  ChResult read_channel(uint8_t ch) {
    ChResult r = {0, 0};
    cs_write_verify(0x08, 0x03);
    cs_write_verify(0x07, ch);
    cs_write_reg(0x00, 0xCF);
    delay(1);
    r.raw = cs_sample_avg(20, 1000);
    r.ohm = adc_to_ohm(r.raw);
    append_log_("ch_%02X raw=%lu ohm=%u\n", ch, (unsigned long)r.raw, r.ohm);
    cs_write_reg(0x00, 0xCA);
    delay(2);
    return r;
  }

  static uint16_t half2(uint16_t a, uint16_t b) {
    return (uint16_t)(((uint32_t)a + b) / 2);
  }
  static uint16_t half_s(int32_t v) {
    return v <= 0 ? 0 : (v > 131070 ? 65535 : (uint16_t)(v / 2));
  }

  // --- Pins ---
  uint8_t pin_cs_{16};
  uint8_t pin_clk_{14};
  uint8_t pin_dio_{12};

  // --- Config ---
  float z_scale_{1.0f};
  float z_offset_{0.0f};
  uint16_t cal_offset_{1100};

  // --- Scan state ---
  bool scan_ok_{false};
  bool ready_{false};
  bool body_valid_{false};
  uint32_t scan_duration_ms_{0};
  uint32_t short_adc_{0}, cal_a_{0}, cal_b_{0};
  float z_eff_ohm_{0};
  uint16_t body_res_{0};
  uint16_t z1_{0}, z2_{0}, z3_{0}, z4_{0}, z5_{0};

  // Channel results
  ChResult lr_leg_{}, rl_leg_{}, lr_hand_{};
  ChResult r_leg_{}, l_leg_{};
  ChResult lh_ll_{}, lh_rl_{}, rh_ll_{}, rh_rl_{};

  // Sensors
  sensor::Sensor *impedance_sensor_{nullptr};
  sensor::Sensor *body_res_sensor_{nullptr};
  sensor::Sensor *lr_leg_sensor_{nullptr};
  sensor::Sensor *rl_leg_sensor_{nullptr};
  sensor::Sensor *lr_hand_sensor_{nullptr};
  sensor::Sensor *r_leg_sensor_{nullptr};
  sensor::Sensor *l_leg_sensor_{nullptr};
  sensor::Sensor *lh_ll_sensor_{nullptr};
  sensor::Sensor *lh_rl_sensor_{nullptr};
  sensor::Sensor *rh_ll_sensor_{nullptr};
  sensor::Sensor *rh_rl_sensor_{nullptr};
  sensor::Sensor *z1_sensor_{nullptr};
  sensor::Sensor *z2_sensor_{nullptr};
  sensor::Sensor *z3_sensor_{nullptr};
  sensor::Sensor *z4_sensor_{nullptr};
  sensor::Sensor *z5_sensor_{nullptr};
  sensor::Sensor *short_adc_sensor_{nullptr};
  sensor::Sensor *cal_a_sensor_{nullptr};
  sensor::Sensor *cal_b_sensor_{nullptr};
  sensor::Sensor *scan_ms_sensor_{nullptr};
  char scan_log_[600]{};
};

}  // namespace cs1258
}  // namespace esphome
