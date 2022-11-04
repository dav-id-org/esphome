#include "as3935_i2c.h"
#include "esphome/core/log.h"

namespace esphome {
namespace as3935_i2c {

static const char *const TAG = "as3935_i2c";

void I2CAS3935Component::write_register(uint8_t reg, uint8_t mask, uint8_t bits, uint8_t start_pos) {
  uint8_t write_data = bits;

  /* Skip read if mask = 0xFF and start pos = 0 (entire register will be written) */
  if ((0xFF != mask) || (0U != start_pos)) {
    uint8_t read_data;

    if (true != read_byte(reg, &read_data, I2C_WRITE_WITHOUT_STOP)) {
      ESP_LOGW(TAG, "Reading register %X failed!", reg);
      return;
    }

    ESP_LOGV(TAG, "write_register : initial value = %X", read_data);
    write_data = (read_data & (~mask)) | ((bits << start_pos) & mask);
  }

  /* Write register */
  if (true != write_byte(reg, write_data)) {
    ESP_LOGW(TAG, "write register %X failed!", reg);
    return;
  }

  ESP_LOGV(TAG, "write_register : updated value = %X", write_data);
}

uint8_t I2CAS3935Component::read_register(uint8_t reg) {
  uint8_t read_data;

  if (true != read_byte(reg, &read_data, I2C_WRITE_WITHOUT_STOP)) {
    ESP_LOGW(TAG, "Reading register %X failed!", reg);
    return 0U;
  }

  ESP_LOGV(TAG, "Read : value = %X", read_data);
  return read_data;
}

void I2CAS3935Component::dump_config() {
  AS3935Component::dump_config();
  LOG_I2C_DEVICE(this);
}

}  // namespace as3935_i2c
}  // namespace esphome
