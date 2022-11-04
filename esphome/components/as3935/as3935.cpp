#include "as3935.h"
#include "as3935_registers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace as3935 {

static const char *const TAG = "as3935";

void AS3935Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AS3935...");

  this->irq_pin_->setup();
  LOG_PIN("  IRQ Pin: ", this->irq_pin_);

  /* Write properties to sensor */
  /* - settings data */
  this->write_indoor(this->indoor_);
  this->write_noise_level(this->noise_level_);
  this->write_watchdog_threshold(this->watchdog_threshold_);
  this->write_spike_rejection(this->spike_rejection_);
  this->write_lightning_threshold(this->lightning_threshold_);
  this->write_mask_disturber(this->mask_disturber_);
  this->write_capacitance(this->capacitance_);

  /* - If mode NORMAL_AUTOCAL is active, launch SRCO and TRCO calibration
   * (warning accuracy of calibration depend of 500kHz tuning of antenna)
   */
  if (AS3935_DEVICE_MODE_NORMAL_AUTOCAL == this->device_mode_) {
    ESP_LOGD(TAG, "launch SRCO and TRCO auto calibration");
    this->write_register(0x3D, 0xFF, 0x96, 0U);
  }

  /* - Set device mode */
  this->write_device_mode(this->device_mode_);

  /* initialize binary sensor */
  if (nullptr != this->thunder_alert_binary_sensor_) {
    this->thunder_alert_binary_sensor_->publish_state(false);
  }
  /* initialize faults state sensor */
  if (nullptr != this->faults_state_sensor_) {
    this->faults_state_sensor_->publish_state(0U);
  }
  /* initialize distance */
  if (nullptr != this->distance_sensor_) {
    this->distance_sensor_->publish_state(63U);
  }
  /* initialize lightening energy */
  if (nullptr != this->energy_sensor_) {
    this->energy_sensor_->publish_state(0U);
  }
}

void AS3935Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AS3935:");
  LOG_PIN("  Interrupt Pin: ", this->irq_pin_);
  LOG_BINARY_SENSOR("  ", "Thunder alert", this->thunder_alert_binary_sensor_);
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_SENSOR("  ", "Lightning energy", this->energy_sensor_);

  /* Read AS3935 registers */
  for (uint8_t wI = 0U; wI <= 8U; ++wI) {
    ESP_LOGD(TAG, "AS3935 register 0x%X value = %X", wI, this->read_data_(wI));
  }

  ESP_LOGD(TAG, "AS3935 register 0x3A value = %X", this->read_data_(0x3A));
  ESP_LOGD(TAG, "AS3935 register 0x3B value = %X", this->read_data_(0x3B));

  /* reset previous INT value (to have log messages when log is launched) */
  this->previous_int_value = 0U;
}

float AS3935Component::get_setup_priority() const { return setup_priority::DATA; }

void AS3935Component::loop() {
  /* read is only made if IRQ pin is high */
  if (true == this->irq_pin_->digital_read()) {
    /* get INT status data */
    uint8_t w_int_value = this->get_interrupt_status_() & INT_ALL_FLAGS_MASK;
    /* calculate difference between actual and previous value with XOR */
    uint8_t w_int_bits_delta = w_int_value ^ this->previous_int_value;

    /* Test if lightening was detected */
    if (8U == w_int_value) {
      ESP_LOGI(TAG, "Lightning has been detected");
      /* activate binary sensor */
      if (nullptr != this->thunder_alert_binary_sensor_) {
        this->thunder_alert_binary_sensor_->publish_state(true);
      }
      /* update distance */
      if (nullptr != this->distance_sensor_) {
        this->distance_sensor_->publish_state(this->get_distance_to_storm_());
      }
      /* update lightening energy */
      if (nullptr != this->energy_sensor_) {
        this->energy_sensor_->publish_state(this->get_lightning_energy_());
      }
    }
    /* Test if historical data are purged */
    else if (0U == w_int_value) {
      ESP_LOGI(TAG, "Purge old data");
      /* activate binary sensor */
      if (nullptr != this->thunder_alert_binary_sensor_) {
        this->thunder_alert_binary_sensor_->publish_state(false);
      }
      /* update distance */
      if (nullptr != this->distance_sensor_) {
        this->distance_sensor_->publish_state(this->get_distance_to_storm_());
      }
      /* update lightening energy */
      if (nullptr != this->energy_sensor_) {
        this->energy_sensor_->publish_state(0U);
      }
    }
    /* else do nothing */

    /* Test each disturbers independently */
    /* Noise level */
    if (0U != (w_int_bits_delta & INT_NH_FLAG_POS)) {
      if (0U != (w_int_value & INT_NH_FLAG_POS)) {
        ESP_LOGW(TAG, "Noise was detected - try increasing the noise level value!");
      } else {
        ESP_LOGI(TAG, "Noise level goes back to normal value");
      }
    }
    /* Spikes */
    if (0U != (w_int_bits_delta & INT_D_FLAG_POS)) {
      if (0U != (w_int_value & INT_D_FLAG_POS)) {
        ESP_LOGW(TAG, "Disturber was detected - try increasing the spike rejection value!");
      } else {
        ESP_LOGI(TAG, "Disturber causes disappeared");
      }
    }

    /* Update faults state sensor in function of INT register :
     * faults state sensor bit 0 : noise level
     * faults state sensor bit 1 : spikes
     */
    if (nullptr != this->faults_state_sensor_) {
      uint8_t w_faults_state = (w_int_value & INT_NH_FLAG_POS) | ((w_int_value & INT_D_FLAG_POS) >> 1U);

      this->faults_state_sensor_->publish_state(w_faults_state);
    }

    /* Update previous value (used to calculate w_int_bits_delta) */
    this->previous_int_value = w_int_value;
  }
}

// REG0x00, bits[5:1], manufacturer default: 10010 (18).
// This setting determines the AFE Gain Boost
void AS3935Component::write_indoor(bool indoor) {
  ESP_LOGV(TAG, "Setting indoor to %d", indoor);
  uint8_t w_afe_gain_val = AFE_GB_VAL_OUTDOOR;

  if (true == indoor) {
    w_afe_gain_val = AFE_GB_VAL_INDOOR;
  }

  this->write_register(AFE_GB_REG_NB, AFE_GB_MASK, w_afe_gain_val, AFE_GB_BIT_POS);
}

// REG0x01, bits[3:0], manufacturer default: 0010 (2).
// This setting determines the threshold for events that trigger the
// IRQ Pin.
void AS3935Component::write_watchdog_threshold(uint8_t watchdog_threshold) {
  ESP_LOGV(TAG, "Setting watchdog sensitivity to %d", watchdog_threshold);
  /* WDTH setting = [0~10] */
  if (watchdog_threshold <= 10U) {
    this->write_register(WDTH_REG_NB, WDTH_MASK, watchdog_threshold, WDTH_BIT_POS);
  }
}

// REG0x01, bits [6:4], manufacturer default: 010 (2).
// The noise floor level is compared to a known reference voltage. If this
// level is exceeded the chip will issue an interrupt to the IRQ pin,
// broadcasting that it can not operate properly due to noise (INT_NH).
// Check datasheet for specific noise level tolerances when setting this register.
void AS3935Component::write_noise_level(uint8_t noise_level) {
  ESP_LOGV(TAG, "Setting noise level to %d", noise_level);
  /* NF_LEV setting = [0~7] */
  if (noise_level <= 7U) {
    this->write_register(NF_LEV_REG_NB, NF_LEV_MASK, noise_level, NF_LEV_BIT_POS);
  }
}

// REG0x02, bits [3:0], manufacturer default: 0010 (2).
// This setting, like the watchdog threshold, can help determine between false
// events and actual lightning. The shape of the spike is analyzed during the
// chip's signal validation routine. Increasing this value increases robustness
// at the cost of sensitivity to distant events.
void AS3935Component::write_spike_rejection(uint8_t spike_rejection) {
  ESP_LOGV(TAG, "Setting spike rejection to %d", spike_rejection);
  /* SREJ setting = [0~11] */
  if (spike_rejection <= 11U) {
    this->write_register(SREJ_REG_NB, SREJ_MASK, spike_rejection, SREJ_BIT_POS);
  }
}

// REG0x02, bits [5:4], manufacturer default: 0 (single lightning strike).
// The number of lightning events before IRQ is set high. 15 minutes is The
// window of time before the number of detected lightning events is reset.
// The number of lightning strikes can be set to 1,5,9, or 16.
void AS3935Component::write_lightning_threshold(uint8_t lightning_threshold) {
  ESP_LOGV(TAG, "Setting lightning threshold to %d", lightning_threshold);
  uint8_t w_min_num_ligh = 0U;
  /* MIN_NUM_LIGH : 1 -> 0, 5 -> 1, 9 -> 2, 16 -> 3 */
  switch (lightning_threshold) {
    case 1U:
      w_min_num_ligh = 0U;
      break;
    case 5U:
      w_min_num_ligh = 1U;
      break;
    case 9U:
      w_min_num_ligh = 2U;
      break;
    case 16U:
      w_min_num_ligh = 3U;
      break;
    default:
      ESP_LOGE(TAG, "Setting lightning threshold, data not equal to [1,5,9,16]");
      break;
  }

  this->write_register(MIN_NUM_LIGH_REG_NB, MIN_NUM_LIGH_MASK, w_min_num_ligh, MIN_NUM_LIGH_BIT_POS);
}

// REG0x03, bit [5], manufacturer default: 0.
// This setting will return whether or not disturbers trigger the IRQ Pin.
void AS3935Component::write_mask_disturber(bool enabled) {
  ESP_LOGV(TAG, "Setting mask disturber to %d", enabled);
  uint8_t w_mask_dist = 0U;

  if (true == enabled) {
    w_mask_dist = 1U;
  }

  this->write_register(MASK_DIST_REG_NB, MASK_DIST_MASK, w_mask_dist, MASK_DIST_BIT_POS);
}

// REG0x03, bit [7:6], manufacturer default: 0 (16 division ratio).
// The antenna is designed to resonate at 500kHz and so can be tuned with the
// following setting. The accuracy of the antenna must be within 3.5 percent of
// that value for proper signal validation and distance estimation.
void AS3935Component::write_div_ratio(uint8_t div_ratio) {
  ESP_LOGV(TAG, "Setting div ratio to %d", div_ratio);
  uint8_t w_div_ratio = 0U;

  switch (div_ratio) {
    case 16U:
      w_div_ratio = 0U;
      break;
    case 32U:
      w_div_ratio = 1U;
      break;
    case 64U:
      w_div_ratio = 2U;
      break;
    case 128U:
      w_div_ratio = 3U;
      break;
    default:
      ESP_LOGE(TAG, "Setting div ratio, data not equal to [16,32,64,128]");
      break;
  }

  this->write_register(LCO_FDIV_REG_NB, LCO_FDIV_MASK, w_div_ratio, LCO_FDIV_BIT_POS);
}

/* REG0x08, bits [7,6,5], manufacturer default: 0U (IRQ pin used for normal operation).
 *
 * DISP_xxCO contain 3 flags : DISP_LCO, DISP_SRCO and DISP_TRCO, that can be used to
 * route corresponding clock on IRQ pin, for calibration.
 *
 * The antenna is designed to resonate at 500kHz and so can be tuned with DISP_LCO.
 * The accuracy of the antenna must be within 3.5 percent of
 * that value for proper signal validation and distance estimation.
 */
void AS3935Component::write_device_mode(AS3935DeviceMode device_mode) {
  ESP_LOGV(TAG, "Setting device mode to %d", device_mode);
  uint8_t w_xxco_value = 0U;

  switch (device_mode) {
    case AS3935_DEVICE_MODE_NORMAL:
    case AS3935_DEVICE_MODE_NORMAL_AUTOCAL:
    default:
      w_xxco_value |= 0U;
      break;
    case AS3935_DEVICE_MODE_LCO_CALIBRATION:
      w_xxco_value |= 4U;
      break;
    case AS3935_DEVICE_MODE_SRCO_DISPLAY:
      w_xxco_value |= 2U;
      break;
    case AS3935_DEVICE_MODE_TRCO_DISPLAY:
      w_xxco_value |= 1U;
      break;
  }

  this->write_register(DISP_xxCO_REG_NB, DISP_xxCO_MASK, w_xxco_value, DISP_xxCO_BIT_POS);
}

// REG0x08, bits [3:0], manufacturer default: 0.
// This setting will add capacitance to the series RLC antenna on the product
// to help tune its resonance. The datasheet specifies being within 3.5 percent
// of 500kHz to get optimal lightning detection and distance sensing.
// It's possible to add up to 120pF in steps of 8pF to the antenna.
void AS3935Component::write_capacitance(uint8_t capacitance) {
  ESP_LOGV(TAG, "Setting tune cap to %d pF", capacitance * 8U);

  this->write_register(DISP_TUN_CAP_REG_NB, TUN_CAP_MASK, capacitance, TUN_CAP_BIT_POS);
}

// REG0x02, bit [6], manufacturer default: 1.
// This register clears the number of lightning strikes that has been read in
// the last 15 minute block.
void AS3935Component::clear_statistics_() {
  // Write high, then low, then high to clear.
  ESP_LOGV(TAG, "Calling clear_statistics_");

  this->write_register(CL_STAT_REG_NB, CL_STAT_MASK, 1U, CL_STAT_BIT_POS);
  this->write_register(CL_STAT_REG_NB, CL_STAT_MASK, 0U, CL_STAT_BIT_POS);
  this->write_register(CL_STAT_REG_NB, CL_STAT_MASK, 1U, CL_STAT_BIT_POS);
}

// REG0x03, bits [3:0], manufacturer default: 0.
// When there is an event that exceeds the watchdog threshold, the register is written
// with the type of event. This consists of two messages: INT_D (disturber detected) and
// INT_L (Lightning detected). A third interrupt INT_NH (noise level too HIGH)
// indicates that the noise level has been exceeded and will persist until the
// noise has ended. Events are active HIGH. There is a one second window of time to
// read the interrupt register after lightning is detected, and 1.5 after
// disturber.
uint8_t AS3935Component::get_interrupt_status_() {
  // A 2ms delay is added to allow for the memory register to be populated
  // after the interrupt pin goes HIGH. See "Interrupt Management" in
  // datasheet.
  // ESP_LOGV(TAG, "Calling read_interrupt_register_");

  delay(2U);
  return (this->read_data_(INT_REG_NB, INT_MASK));
}

// REG0x07, bit [5:0], manufacturer default: 0.
// This register holds the distance to the front of the storm and not the
// distance to a lightning strike.
uint8_t AS3935Component::get_distance_to_storm_() {
  ESP_LOGV(TAG, "Calling get_distance_to_storm_");

  return (this->read_data_(DISTANCE_REG_NB, DISTANCE_MASK));
}

uint32_t AS3935Component::get_lightning_energy_() {
  ESP_LOGV(TAG, "Calling get_lightning_energy_");
  /* Variable for lightning energy which is just a pure number. */
  uint32_t pure_light = (this->read_data_(S_SIG_MM_REG_NB, S_SIG_MM_MASK) << 16U) +
                        (this->read_data_(S_SIG_M_REG_NB) << 8U) + this->read_data_(S_SIG_L_REG_NB);

  return (pure_light);
}

uint8_t AS3935Component::read_data_(uint8_t reg, uint8_t mask, uint8_t bit_pos) {
  return ((this->read_register(reg) & mask) >> bit_pos);
}

}  // namespace as3935
}  // namespace esphome
