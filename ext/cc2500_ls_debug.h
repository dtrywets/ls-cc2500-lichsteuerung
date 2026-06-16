#pragma once
#include <cstring>
#include <SPI.h>
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace cc2500_ls {

static const char *TAG = "cc2500_dbg";

static const uint8_t CC2500_SRES = 0x30;
static const uint8_t CC2500_SRX = 0x34;
static const uint8_t CC2500_SIDLE = 0x36;
static const uint8_t CC2500_SFRX = 0x3A;
static const uint8_t CC2500_FIFO = 0x3F;
static const uint8_t REG_PARTNUM = 0x30;
static const uint8_t REG_VERSION = 0x31;
static const uint8_t REG_RSSI = 0x34;
static const uint8_t REG_MARCSTATE = 0x35;
static const uint8_t REG_RXBYTES = 0x3B;
static const uint8_t REG_PATABLE = 0x3E;

class CC2500LSDebug : public Component {
 public:
  void set_pins(uint8_t cs, uint8_t pa_en, uint8_t rx_en) {
    cs_ = cs;
    pa_en_ = pa_en;
    rx_en_ = rx_en;
  }

  void set_spi_pins(uint8_t sck, uint8_t miso, uint8_t mosi) {
    sck_ = sck;
    miso_ = miso;
    mosi_ = mosi;
    spi_pins_set_ = true;
  }

  void set_text_sensors(text_sensor::TextSensor *chip, text_sensor::TextSensor *status,
                        text_sensor::TextSensor *last_rx) {
    chip_ts_ = chip;
    status_ts_ = status;
    last_rx_ts_ = last_rx;
  }

  void set_sensors(sensor::Sensor *rssi, sensor::Sensor *channel, sensor::Sensor *rx_count) {
    rssi_s_ = rssi;
    channel_s_ = channel;
    rx_count_s_ = rx_count;
  }

  void set_chip_ok(binary_sensor::BinarySensor *bs) { chip_ok_bs_ = bs; }

  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void setup() override {
    ESP_LOGI(TAG, "Setup start (CS=%u PA_EN=%u RX_EN=%u)", cs_, pa_en_, rx_en_);
    pinMode(cs_, OUTPUT);
    digitalWrite(cs_, HIGH);
    pinMode(pa_en_, OUTPUT);
    pinMode(rx_en_, OUTPUT);
    set_rx_mode_();
    start_scan_(false);
  }

  void loop() override {
    if (scanning_) {
      ScanResult r = tick_scan_();
      if (r == ScanResult::FOUND) {
        scanning_ = false;
        chip_ok_ = true;
        if (chip_ok_bs_ != nullptr)
          chip_ok_bs_->publish_state(true);
        init_profile_();
        enter_rx_();
        publish_chip_info_();
        publish_status_("OK: RX aktiv, Kanal-Scan laeuft");
        ESP_LOGI(TAG, "CC2500 initialisiert, Kanal=%u", channel_);
      } else if (r == ScanResult::FAILED) {
        scanning_ = false;
        chip_ok_ = false;
        if (chip_ok_bs_ != nullptr)
          chip_ok_bs_->publish_state(false);
        ESP_LOGE(TAG, "CC2500 nicht erkannt");
      }
      return;
    }

    if (!chip_ok_) {
      if (millis() - last_status_ms_ > 15000) {
        last_status_ms_ = millis();
        ESP_LOGW(TAG, "CC2500 nicht bereit — PARTNUM=0x%02X VER=0x%02X (erwarte 0x80/0x04)", partnum_, version_);
      }
      return;
    }

    poll_rx_();

    uint32_t now = millis();
    if (rx_count_ == 0 && now - last_channel_hop_ms_ > 3000) {
      last_channel_hop_ms_ = now;
      hop_channel_();
    }
    if (now - last_status_ms_ > 10000) {
      last_status_ms_ = now;
      log_status_();
    }
  }

  void reinit() {
    if (scanning_) {
      ESP_LOGW(TAG, "Scan laeuft bereits");
      return;
    }
    if (chip_ok_) {
      ESP_LOGI(TAG, "Soft-Reinit (Register)...");
      init_profile_();
      enter_rx_();
      publish_chip_info_();
      publish_status_("Neuinitialisiert (Register)");
      return;
    }
    ESP_LOGI(TAG, "Reinit mit Quick-Scan...");
    start_scan_(false);
  }

  void start_full_scan() {
    if (scanning_) {
      ESP_LOGW(TAG, "Scan laeuft bereits");
      return;
    }
    ESP_LOGI(TAG, "Voll-Scan gestartet (nicht blockierend)...");
    chip_ok_ = false;
    if (chip_ok_bs_ != nullptr)
      chip_ok_bs_->publish_state(false);
    start_scan_(true);
  }

  bool is_chip_ok() const { return chip_ok_; }
  bool is_scanning() const { return scanning_; }
  uint32_t get_rx_count() const { return rx_count_; }
  uint8_t get_channel() const { return channel_; }
  uint8_t get_partnum() const { return partnum_; }
  uint8_t get_version() const { return version_; }
  const char *get_probe_summary() const { return probe_summary_; }

  enum class ScanResult { RUNNING, FOUND, FAILED };

 protected:
  uint8_t partnum_{0};
  uint8_t version_{0};
  uint8_t cs_{15};
  uint8_t pa_en_{16};
  uint8_t rx_en_{0};
  uint8_t sck_{14};
  uint8_t miso_{12};
  uint8_t mosi_{13};
  bool spi_pins_set_{false};
  bool chip_ok_{false};
  bool spi_slow_{false};
  bool spi_lsb_{false};
  uint8_t spi_mode_{0};
  bool pa_rx_inverted_{false};
  char probe_summary_[240]{0};
  uint8_t channel_{0x10};
  uint8_t channel_idx_{0};
  uint32_t rx_count_{0};
  uint32_t last_channel_hop_ms_{0};
  uint32_t last_status_ms_{0};
  bool scanning_{false};
  bool scan_full_{false};
  size_t scan_pi_{0};
  size_t scan_mi_{0};
  uint8_t scan_mode_i_{0};
  uint8_t scan_si_{0};
  uint8_t scan_li_{0};
  uint8_t scan_hw_i_{0};
  uint8_t scan_best_part_{0};
  uint8_t scan_best_ver_{0};
  uint32_t scan_steps_done_{0};

  static const uint8_t SCAN_CHANNELS[12];

  text_sensor::TextSensor *chip_ts_{nullptr};
  text_sensor::TextSensor *status_ts_{nullptr};
  text_sensor::TextSensor *last_rx_ts_{nullptr};
  sensor::Sensor *rssi_s_{nullptr};
  sensor::Sensor *channel_s_{nullptr};
  sensor::Sensor *rx_count_s_{nullptr};
  binary_sensor::BinarySensor *chip_ok_bs_{nullptr};

  bool use_hw_spi_{false};

  void init_spi_(uint8_t sck, uint8_t miso, uint8_t mosi) {
    if (use_hw_spi_)
      SPI.end();
    use_hw_spi_ = false;
    sck_ = sck;
    miso_ = miso;
    mosi_ = mosi;
    pinMode(sck_, OUTPUT);
    pinMode(mosi_, OUTPUT);
    pinMode(miso_, INPUT_PULLUP);
    digitalWrite(sck_, LOW);
    digitalWrite(mosi_, LOW);
    delay(10);
  }

  void init_hw_spi_() {
    use_hw_spi_ = true;
    sck_ = 14;
    miso_ = 12;
    mosi_ = 13;
    SPI.begin();
    SPI.setFrequency(1000000);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    delay(10);
  }

  void start_scan_(bool full_scan) {
    if (spi_pins_set_) {
      scanning_ = true;
      scan_full_ = false;
      scan_pi_ = 999;
      publish_status_("SPI Scan laeuft...");
      return;
    }
    scan_full_ = full_scan;
    scanning_ = true;
    scan_pi_ = 0;
    scan_mi_ = 0;
    scan_mode_i_ = 0;
    scan_si_ = 0;
    scan_li_ = 0;
    scan_hw_i_ = 0;
    scan_steps_done_ = 0;
    scan_best_part_ = 0;
    scan_best_ver_ = 0;
    probe_summary_[0] = '\0';
    publish_status_(full_scan ? "SPI Voll-Scan laeuft (HA bleibt online)..." : "SPI Quick-Scan laeuft...");
  }

  size_t scan_pa_count_() const { return scan_full_ ? 6u : 2u; }
  size_t scan_map_count_() const { return scan_full_ ? 6u : 2u; }

  void get_scan_pa_(size_t pi, int8_t &pa, int8_t &rx, const char *&name) const {
    struct PaRxMode {
      const char *n;
      int8_t pa;
      int8_t rx;
    };
    static const PaRxMode full[] = {
        {"PA=0 RX=1", 0, 1}, {"PA=1 RX=0", 1, 0}, {"PA=0 RX=0", 0, 0},
        {"PA=1 RX=1", 1, 1}, {"PA=float RX=float", -1, -1}, {"PA=0 RX=float", 0, -1},
    };
    static const PaRxMode quick[] = {{"PA=0 RX=1", 0, 1}, {"PA=float RX=float", -1, -1}};
    const PaRxMode &m = scan_full_ ? full[pi] : quick[pi];
    pa = m.pa;
    rx = m.rx;
    name = m.n;
  }

  void get_scan_map_(size_t mi, const char *&name, uint8_t &sck, uint8_t &miso, uint8_t &mosi) const {
    struct SpiMap {
      const char *n;
      uint8_t sck;
      uint8_t miso;
      uint8_t mosi;
    };
    static const SpiMap full[] = {
        {"SCK=D5 MISO=D6 MOSI=D7", 14, 12, 13}, {"SCK=D5 MISO=D7 MOSI=D6", 14, 13, 12},
        {"SCK=D6 MISO=D5 MOSI=D7", 12, 14, 13}, {"SCK=D6 MISO=D7 MOSI=D5", 12, 13, 14},
        {"SCK=D7 MISO=D5 MOSI=D6", 13, 14, 12}, {"SCK=D7 MISO=D6 MOSI=D5", 13, 12, 14},
    };
    static const SpiMap quick[] = {
        {"SCK=D5 MISO=D6 MOSI=D7", 14, 12, 13},
        {"SCK=D5 MISO=D7 MOSI=D6", 14, 13, 12},
    };
    const SpiMap &m = scan_full_ ? full[mi] : quick[mi];
    name = m.n;
    sck = m.sck;
    miso = m.miso;
    mosi = m.mosi;
  }

  ScanResult tick_scan_() {
    if (spi_pins_set_) {
      apply_pa_rx_(0, 1);
      init_spi_(sck_, miso_, mosi_);
      if (detect_chip_())
        return ScanResult::FOUND;
      publish_status_("FEHLER: CC2500 nicht erkannt (feste SPI-Pins)");
      return ScanResult::FAILED;
    }

    const size_t pa_count = scan_pa_count_();
    if (scan_pi_ >= pa_count)
      return finish_scan_failed_();

    int8_t pa;
    int8_t rx;
    const char *pa_name;
    get_scan_pa_(scan_pi_, pa, rx, pa_name);
    apply_pa_rx_(pa, rx);

    const bool do_hw = !scan_full_ || scan_pi_ == 0;
    if (do_hw && scan_hw_i_ < 2) {
      init_hw_spi_();
      spi_mode_ = scan_hw_i_;
      SPI.setDataMode(scan_hw_i_ == 0 ? SPI_MODE0 : SPI_MODE1);
      ESP_LOGI(TAG, "Probe %s / hw_spi mode%u", pa_name, scan_hw_i_);
      if (detect_chip_()) {
        char summary[96];
        snprintf(summary, sizeof(summary), "OK: %s | hw_spi | mode%u", pa_name, scan_hw_i_);
        probe_summary_[0] = '\0';
        strncpy(probe_summary_, summary, sizeof(probe_summary_) - 1);
        publish_status_(probe_summary_);
        return ScanResult::FOUND;
      }
      record_probe_(pa_name, "hw_spi", scan_hw_i_, 0, scan_best_part_, scan_best_ver_);
      scan_hw_i_++;
      scan_steps_done_++;
      if (scan_steps_done_ % 4 == 0) {
        char progress[48];
        snprintf(progress, sizeof(progress), "Scan... %u Schritte", scan_steps_done_);
        publish_status_(progress);
      }
      ESP.wdtFeed();
      yield();
      return ScanResult::RUNNING;
    }

    const size_t map_count = scan_map_count_();
    if (scan_mi_ >= map_count) {
      scan_pi_++;
      scan_mi_ = 0;
      scan_mode_i_ = 0;
      scan_si_ = 0;
      scan_li_ = 0;
      scan_hw_i_ = 0;
      return ScanResult::RUNNING;
    }

    const char *map_name;
    uint8_t sck;
    uint8_t miso;
    uint8_t mosi;
    get_scan_map_(scan_mi_, map_name, sck, miso, mosi);
    const uint8_t mode_max = scan_full_ ? 2u : 1u;
    const uint8_t slow_max = scan_full_ ? 2u : 1u;
    const uint8_t lsb_max = scan_full_ ? 2u : 1u;

    spi_mode_ = scan_mode_i_;
    spi_slow_ = scan_si_ != 0;
    spi_lsb_ = scan_li_ != 0;
    init_spi_(sck, miso, mosi);
    ESP_LOGI(TAG, "Probe %s / %s / m%u / %s / %s", pa_name, map_name, scan_mode_i_,
             spi_slow_ ? "slow" : "fast", spi_lsb_ ? "lsb" : "msb");
      if (detect_chip_()) {
        char summary[128];
        snprintf(summary, sizeof(summary), "OK: %s | %s | m%u %s %s", pa_name, map_name, scan_mode_i_,
                 spi_slow_ ? "slow" : "fast", spi_lsb_ ? "lsb" : "msb");
        strncpy(probe_summary_, summary, sizeof(probe_summary_) - 1);
        probe_summary_[sizeof(probe_summary_) - 1] = '\0';
        publish_status_(probe_summary_);
        return ScanResult::FOUND;
      }
      record_probe_(pa_name, map_name, scan_mode_i_, spi_slow_, scan_best_part_, scan_best_ver_);
      scan_steps_done_++;
      if (scan_steps_done_ % 8 == 0) {
        char progress[48];
        snprintf(progress, sizeof(progress), "Scan... %u Schritte", scan_steps_done_);
        publish_status_(progress);
      }

    scan_li_++;
    if (scan_li_ >= lsb_max) {
      scan_li_ = 0;
      scan_si_++;
      if (scan_si_ >= slow_max) {
        scan_si_ = 0;
        scan_mode_i_++;
        if (scan_mode_i_ >= mode_max) {
          scan_mode_i_ = 0;
          scan_mi_++;
        }
      }
    }
    ESP.wdtFeed();
    yield();
    return ScanResult::RUNNING;
  }

  ScanResult finish_scan_failed_() {
    char status[280];
    partnum_ = scan_best_part_;
    version_ = scan_best_ver_;
    if (!scan_full_) {
      snprintf(status, sizeof(status), "FEHLER: Quick-Scan — SPI Scan fuer Volltest | best PART=0x%02X VER=0x%02X",
               scan_best_part_, scan_best_ver_);
    } else {
      snprintf(status, sizeof(status), "Kein Treffer:%s | best PART=0x%02X VER=0x%02X", probe_summary_,
               scan_best_part_, scan_best_ver_);
    }
    publish_status_(status);
    ESP_LOGW(TAG, "Probe failed: %s", status);
    return ScanResult::FAILED;
  }

  bool probe_all_(bool full_scan) {
    start_scan_(full_scan);
    while (scanning_) {
      ScanResult r = tick_scan_();
      if (r == ScanResult::FOUND) {
        scanning_ = false;
        return true;
      }
      if (r == ScanResult::FAILED) {
        scanning_ = false;
        return false;
      }
    }
    return false;
  }

  void record_probe_(const char *pa, const char *map, uint8_t mode, bool slow, uint8_t &best_part,
                     uint8_t &best_ver) {
    if (partnum_ == 0x80)
      return;
    char line[80];
    snprintf(line, sizeof(line), "[%s+%s m%u%c=%02X/%02X] ", pa, map, mode, slow ? 's' : 'f', partnum_,
             version_);
    if (strlen(probe_summary_) < sizeof(probe_summary_) - 40)
      strncat(probe_summary_, line, sizeof(probe_summary_) - strlen(probe_summary_) - 1);
    // 0x80 ideal; 0x0F oft MISO-Float — naeher an erwartetem Wert merken
    if (partnum_ == 0x80 || (best_part != 0x80 && partnum_ > best_part)) {
      best_part = partnum_;
      best_ver = version_;
    }
  }

  void apply_pa_rx_(int8_t pa, int8_t rx) {
    if (pa < 0) {
      pinMode(pa_en_, INPUT_PULLUP);
    } else {
      pinMode(pa_en_, OUTPUT);
      digitalWrite(pa_en_, pa ? HIGH : LOW);
    }
    if (rx < 0) {
      pinMode(rx_en_, INPUT_PULLUP);
    } else {
      pinMode(rx_en_, OUTPUT);
      digitalWrite(rx_en_, rx ? HIGH : LOW);
    }
    delay(10);
  }

  bool probe_spi_() { return probe_all_(true); }

  void set_rx_mode_() { apply_pa_rx_(0, 1); }

  uint8_t spi_delay_us_() const { return spi_slow_ ? 20 : 2; }

  uint8_t xfer_hw_(uint8_t v) {
    if (spi_mode_ == 0) {
      SPI.setDataMode(SPI_MODE0);
    } else {
      SPI.setDataMode(SPI_MODE1);
    }
    return SPI.transfer(v);
  }

  uint8_t xfer_bb_(uint8_t v) {
    uint8_t r = 0;
    const uint8_t d = spi_delay_us_();
    if (spi_lsb_) {
      for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(mosi_, (v >> i) & 1);
        delayMicroseconds(d);
        digitalWrite(sck_, HIGH);
        delayMicroseconds(d);
        if (digitalRead(miso_))
          r |= (1 << i);
        digitalWrite(sck_, LOW);
        delayMicroseconds(d);
        yield();
      }
    } else {
      for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(mosi_, (v >> i) & 1);
        delayMicroseconds(d);
        digitalWrite(sck_, HIGH);
        delayMicroseconds(d);
        if (digitalRead(miso_))
          r |= (1 << i);
        digitalWrite(sck_, LOW);
        delayMicroseconds(d);
        yield();
      }
    }
    return r;
  }

  void xfer_begin_() {
    digitalWrite(cs_, LOW);
    delayMicroseconds(2);
  }

  void xfer_end_() {
    delayMicroseconds(2);
    digitalWrite(cs_, HIGH);
  }

  uint8_t xfer_(uint8_t v) {
    if (use_hw_spi_)
      return xfer_hw_(v);
    return xfer_bb_(v);
  }

  uint8_t read_reg_(uint8_t addr) {
    xfer_begin_();
    uint8_t status = xfer_((uint8_t)(addr | 0x80));
    delayMicroseconds(spi_slow_ ? 50 : 10);
    uint8_t v = xfer_(0x00);
    xfer_end_();
    delayMicroseconds(50);
    (void) status;
    return v;
  }

  void write_reg_(uint8_t addr, uint8_t value) {
    xfer_begin_();
    xfer_(addr);
    delayMicroseconds(200);
    xfer_(value);
    xfer_end_();
    delayMicroseconds(50);
  }

  void strobe_(uint8_t strobe) {
    xfer_begin_();
    xfer_(strobe);
    xfer_end_();
    delayMicroseconds(2000);
  }

  bool detect_chip_() {
    // CS sicher HIGH, dann Reset
    digitalWrite(cs_, HIGH);
    delay(5);
    strobe_(CC2500_SRES);
    delay(200);

    uint8_t part = 0;
    uint8_t ver = 0;
    uint8_t marc = 0;
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
      part = read_reg_(REG_PARTNUM);
      ver = read_reg_(REG_VERSION);
      marc = read_reg_(REG_MARCSTATE);
      if (part == 0x80)
        break;
      delay(20);
    }
    partnum_ = part;
    version_ = ver;
    ESP_LOGI(TAG, "PARTNUM=0x%02X VERSION=0x%02X MARC=0x%02X (erwarte 0x80/0x04)", part, ver, marc);
    if (chip_ts_ != nullptr) {
      char info[32];
      snprintf(info, sizeof(info), "PARTNUM=0x%02X VER=0x%02X", part, ver);
      chip_ts_->publish_state(info);
    }
    return part == 0x80;
  }

  void init_profile_() {
    write_reg_(0x00, 0x29);
    write_reg_(0x02, 0x06);
    write_reg_(0x06, 0xFF);
    write_reg_(0x07, 0x04);
    write_reg_(0x08, 0x05);
    write_reg_(0x09, 0x01);
    write_reg_(0x0A, channel_);
    write_reg_(0x0B, 0x09);
    write_reg_(0x0C, 0x00);
    write_reg_(0x0D, 0x5D);
    write_reg_(0x0E, 0x93);
    write_reg_(0x0F, 0xB1);
    write_reg_(0x10, 0x2D);
    write_reg_(0x11, 0x3B);
    write_reg_(0x12, 0x73);
    write_reg_(0x13, 0xA2);
    write_reg_(0x14, 0xF8);
    write_reg_(0x15, 0x01);
    write_reg_(0x16, 0x07);
    write_reg_(0x17, 0x30);
    write_reg_(0x18, 0x18);
    write_reg_(0x19, 0x1D);
    write_reg_(0x1A, 0x1C);
    write_reg_(0x1B, 0xC7);
    write_reg_(0x1C, 0x00);
    write_reg_(0x1D, 0xB2);
    write_reg_(0x1E, 0x87);
    write_reg_(0x1F, 0x6B);
    write_reg_(0x20, 0xF8);
    write_reg_(0x21, 0xB6);
    write_reg_(0x22, 0x10);
    write_reg_(0x23, 0xEA);
    write_reg_(0x24, 0x0A);
    write_reg_(0x25, 0x00);
    write_reg_(0x26, 0x11);
    write_reg_(0x27, 0x41);
    write_reg_(0x28, 0x00);
    write_reg_(0x29, 0x59);
    write_reg_(REG_PATABLE, 0xFF);
    strobe_(CC2500_SIDLE);
  }

  void enter_rx_() {
    strobe_(CC2500_SIDLE);
    strobe_(CC2500_SFRX);
    strobe_(CC2500_SRX);
    write_reg_(0x01, 0x01);
    delay(5);
  }

  void poll_rx_() {
    uint8_t marc = read_reg_(REG_MARCSTATE);
    if (marc != 0x0D && marc != 0x0F)
      enter_rx_();

    uint8_t rxb = read_reg_(REG_RXBYTES);
    if ((rxb & 0x7F) == 0)
      return;

    uint8_t len = read_reg_(CC2500_FIFO);
    if (len == 0 || len > 64) {
      strobe_(CC2500_SFRX);
      enter_rx_();
      return;
    }

    char hex[200];
    hex[0] = '\0';
    for (uint8_t i = 0; i < len; i++) {
      char byte[4];
      snprintf(byte, sizeof(byte), "%s%02X", i ? " " : "", read_reg_(CC2500_FIFO));
      strncat(hex, byte, sizeof(hex) - strlen(hex) - 1);
    }

    rx_count_++;
    if (rx_count_s_ != nullptr)
      rx_count_s_->publish_state(rx_count_);

    int8_t rssi_raw = (int8_t) read_reg_(REG_RSSI);
    float rssi_dbm =
        rssi_raw >= 128 ? ((rssi_raw - 256) / 2.0f) - 72.0f : (rssi_raw / 2.0f) - 72.0f;
    if (rssi_s_ != nullptr)
      rssi_s_->publish_state(rssi_dbm);
    if (last_rx_ts_ != nullptr)
      last_rx_ts_->publish_state(hex);

    ESP_LOGI(TAG, "RX ch=%u len=%u rssi=%.0f data=%s", channel_, len, rssi_dbm, hex);

    strobe_(CC2500_SIDLE);
    strobe_(CC2500_SFRX);
    enter_rx_();
  }

  void hop_channel_() {
    channel_idx_ = (uint8_t)((channel_idx_ + 1) % 12);
    channel_ = SCAN_CHANNELS[channel_idx_];
    write_reg_(0x0A, channel_);
    enter_rx_();
    if (channel_s_ != nullptr)
      channel_s_->publish_state(channel_);
    char st[32];
    snprintf(st, sizeof(st), "Scan Kanal %u", channel_);
    publish_status_(st);
  }

  void log_status_() {
    uint8_t marc = read_reg_(REG_MARCSTATE);
    int8_t rssi_raw = (int8_t) read_reg_(REG_RSSI);
    float rssi_dbm =
        rssi_raw >= 128 ? ((rssi_raw - 256) / 2.0f) - 72.0f : (rssi_raw / 2.0f) - 72.0f;
    ESP_LOGI(TAG, "Status: ch=%u marc=0x%02X rssi=%.0f rx=%u", channel_, marc, rssi_dbm, rx_count_);
    if (rssi_s_ != nullptr)
      rssi_s_->publish_state(rssi_dbm);
  }

  void publish_chip_info_() {
    uint8_t part = read_reg_(REG_PARTNUM);
    uint8_t ver = read_reg_(REG_VERSION);
    if (chip_ts_ != nullptr) {
      char info[40];
      snprintf(info, sizeof(info), "OK PARTNUM=0x%02X VER=0x%02X", part, ver);
      chip_ts_->publish_state(info);
    }
  }

  void publish_status_(const char *s) {
    if (status_ts_ != nullptr)
      status_ts_->publish_state(s);
  }
};

const uint8_t CC2500LSDebug::SCAN_CHANNELS[12] = {0, 4, 8, 10, 12, 16, 20, 24, 32, 40, 48, 60};

inline CC2500LSDebug &cc2500_instance() {
  static CC2500LSDebug inst;
  return inst;
}
inline CC2500LSDebug *g_cc2500_dbg = nullptr;

}  // namespace cc2500_ls
}  // namespace esphome
