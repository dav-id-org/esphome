#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace as3935 {

enum AS3935DeviceMode {
  AS3935_DEVICE_MODE_NORMAL = 0U,
  AS3935_DEVICE_MODE_NORMAL_AUTOCAL = 1U,
  AS3935_DEVICE_MODE_LCO_CALIBRATION = 2U,
  AS3935_DEVICE_MODE_SRCO_DISPLAY = 3U,
  AS3935_DEVICE_MODE_TRCO_DISPLAY = 4U,
};

class AS3935Component : public Component {
 public:
  /* overrides */
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void loop() override;

  /* setters */
  void set_irq_pin(GPIOPin *irq_pin) { irq_pin_ = irq_pin; }
  void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
  void set_energy_sensor(sensor::Sensor *energy_sensor) { energy_sensor_ = energy_sensor; }
  void set_faults_state_sensor(sensor::Sensor *faults_state_sensor) { faults_state_sensor_ = faults_state_sensor; }
  void set_thunder_alert_binary_sensor(binary_sensor::BinarySensor *thunder_alert_binary_sensor) {
    thunder_alert_binary_sensor_ = thunder_alert_binary_sensor;
  }
  void set_indoor(bool indoor) { indoor_ = indoor; }
  void set_noise_level(uint8_t noise_level) { noise_level_ = noise_level; }
  void set_watchdog_threshold(uint8_t watchdog_threshold) { watchdog_threshold_ = watchdog_threshold; }
  void set_spike_rejection(uint8_t spike_rejection) { spike_rejection_ = spike_rejection; }
  void set_lightning_threshold(uint8_t lightning_threshold) { lightning_threshold_ = lightning_threshold; }
  void set_mask_disturber(bool mask_disturber) { mask_disturber_ = mask_disturber; }
  void set_device_mode(AS3935DeviceMode mode) { device_mode_ = mode; }
  void set_capacitance(uint8_t capacitance) { capacitance_ = capacitance; }

  /* write parameters */
  void write_indoor(bool indoor);
  void write_noise_level(uint8_t noise_level);
  void write_watchdog_threshold(uint8_t watchdog_threshold);
  void write_spike_rejection(uint8_t write_spike_rejection);
  void write_lightning_threshold(uint8_t lightning_threshold);
  void write_mask_disturber(bool enabled);
  void write_div_ratio(uint8_t div_ratio);
  void write_capacitance(uint8_t capacitance);
  void write_device_mode(AS3935DeviceMode mode);

 protected: /* virtual methods */
  virtual uint8_t read_register(uint8_t reg) = 0;
  virtual void write_register(uint8_t reg, uint8_t mask, uint8_t bits, uint8_t start_position) = 0;

 protected: /* internal methods */
  void clear_statistics_();

  uint8_t get_interrupt_status_();
  uint8_t get_distance_to_storm_();
  uint32_t get_lightning_energy_();

  uint8_t read_data_(uint8_t reg, uint8_t mask = 0xFF, uint8_t bit_pos = 0U);

 protected: /* internal attributes */
  GPIOPin *irq_pin_;

  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *faults_state_sensor_{nullptr};
  binary_sensor::BinarySensor *thunder_alert_binary_sensor_{nullptr};

  bool indoor_{true};
  uint8_t noise_level_{2U};
  uint8_t watchdog_threshold_{2U};
  uint8_t spike_rejection_{2U};
  uint8_t lightning_threshold_{1U};
  bool mask_disturber_{false};
  AS3935DeviceMode device_mode_{AS3935_DEVICE_MODE_NORMAL};
  uint8_t capacitance_{0U};

  uint8_t previous_int_value{0U};
};

}  // namespace as3935
}  // namespace esphome
