#ifdef USE_ESP32

#include "esp32_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace esp32_camera {

static const char *const TAG = "esp32_camera";

/* ---------------- public API (derivated) ---------------- */
void ESP32Camera::setup() {
  global_esp32_camera = this;

  /* initialize time to now */
  this->last_update_ = millis();

  /* initialize camera */
  esp_err_t err = esp_camera_init(&this->config_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
    this->init_error_ = err;
    this->mark_failed();
    return;
  }

  /* initialize camera parameters */
  this->update_camera_parameters();
}

void ESP32Camera::dump_config() {
  auto conf = this->config_;
  ESP_LOGCONFIG(TAG, "ESP32 Camera:");
  ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Internal: %s", YESNO(this->internal_));
  ESP_LOGCONFIG(TAG, "  Data Pins: D0:%d D1:%d D2:%d D3:%d D4:%d D5:%d D6:%d D7:%d", conf.pin_d0, conf.pin_d1,
                conf.pin_d2, conf.pin_d3, conf.pin_d4, conf.pin_d5, conf.pin_d6, conf.pin_d7);
  ESP_LOGCONFIG(TAG, "  VSYNC Pin: %d", conf.pin_vsync);
  ESP_LOGCONFIG(TAG, "  HREF Pin: %d", conf.pin_href);
  ESP_LOGCONFIG(TAG, "  Pixel Clock Pin: %d", conf.pin_pclk);
  ESP_LOGCONFIG(TAG, "  External Clock: Pin:%d Frequency:%u", conf.pin_xclk, conf.xclk_freq_hz);
  ESP_LOGCONFIG(TAG, "  I2C Pins: SDA:%d SCL:%d", conf.pin_sscb_sda, conf.pin_sscb_scl);
  ESP_LOGCONFIG(TAG, "  Reset Pin: %d", conf.pin_reset);
  switch (this->config_.frame_size) {
    case FRAMESIZE_QQVGA:
      ESP_LOGCONFIG(TAG, "  Resolution: 160x120 (QQVGA)");
      break;
    case FRAMESIZE_QCIF:
      ESP_LOGCONFIG(TAG, "  Resolution: 176x155 (QCIF)");
      break;
    case FRAMESIZE_HQVGA:
      ESP_LOGCONFIG(TAG, "  Resolution: 240x176 (HQVGA)");
      break;
    case FRAMESIZE_QVGA:
      ESP_LOGCONFIG(TAG, "  Resolution: 320x240 (QVGA)");
      break;
    case FRAMESIZE_CIF:
      ESP_LOGCONFIG(TAG, "  Resolution: 400x296 (CIF)");
      break;
    case FRAMESIZE_VGA:
      ESP_LOGCONFIG(TAG, "  Resolution: 640x480 (VGA)");
      break;
    case FRAMESIZE_SVGA:
      ESP_LOGCONFIG(TAG, "  Resolution: 800x600 (SVGA)");
      break;
    case FRAMESIZE_XGA:
      ESP_LOGCONFIG(TAG, "  Resolution: 1024x768 (XGA)");
      break;
    case FRAMESIZE_SXGA:
      ESP_LOGCONFIG(TAG, "  Resolution: 1280x1024 (SXGA)");
      break;
    case FRAMESIZE_UXGA:
      ESP_LOGCONFIG(TAG, "  Resolution: 1600x1200 (UXGA)");
      break;
    default:
      break;
  }

  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Setup Failed: %s", esp_err_to_name(this->init_error_));
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  auto st = s->status;
  ESP_LOGCONFIG(TAG, "  JPEG Quality: %u", st.quality);
  // ESP_LOGCONFIG(TAG, "  Framebuffer Count: %u", conf.fb_count);
  ESP_LOGCONFIG(TAG, "  Contrast: %d", st.contrast);
  ESP_LOGCONFIG(TAG, "  Brightness: %d", st.brightness);
  ESP_LOGCONFIG(TAG, "  Saturation: %d", st.saturation);
  ESP_LOGCONFIG(TAG, "  Vertical Flip: %s", ONOFF(st.vflip));
  ESP_LOGCONFIG(TAG, "  Horizontal Mirror: %s", ONOFF(st.hmirror));
  ESP_LOGCONFIG(TAG, "  Special Effect: %u", st.special_effect);
  ESP_LOGCONFIG(TAG, "  White Balance Mode: %u", st.wb_mode);
  // ESP_LOGCONFIG(TAG, "  Auto White Balance: %u", st.awb);
  // ESP_LOGCONFIG(TAG, "  Auto White Balance Gain: %u", st.awb_gain);
  ESP_LOGCONFIG(TAG, "  Auto Exposure Control: %u", st.aec);
  ESP_LOGCONFIG(TAG, "  Auto Exposure Control 2: %u", st.aec2);
  ESP_LOGCONFIG(TAG, "  Auto Exposure Level: %d", st.ae_level);
  ESP_LOGCONFIG(TAG, "  Auto Exposure Value: %u", st.aec_value);
  ESP_LOGCONFIG(TAG, "  AGC: %u", st.agc);
  ESP_LOGCONFIG(TAG, "  AGC Gain: %u", st.agc_gain);
  ESP_LOGCONFIG(TAG, "  Gain Ceiling: %u", st.gainceiling);
  // ESP_LOGCONFIG(TAG, "  BPC: %u", st.bpc);
  // ESP_LOGCONFIG(TAG, "  WPC: %u", st.wpc);
  // ESP_LOGCONFIG(TAG, "  RAW_GMA: %u", st.raw_gma);
  // ESP_LOGCONFIG(TAG, "  Lens Correction: %u", st.lenc);
  // ESP_LOGCONFIG(TAG, "  DCW: %u", st.dcw);
  ESP_LOGCONFIG(TAG, "  Test Pattern: %s", YESNO(st.colorbar));
}

void ESP32Camera::loop() {
  enum { FSM_IDLE = 0U, FSM_RELEASE_PICTURE = 1U, /*FSM_PICTURE_STILL_IN_USE managed by default */ };
  auto shared_ptr_count = this->current_image_.use_count();
  // Set behavior in function of shared_ptr use count :
  switch (shared_ptr_count) {
      // 0 : New image can be retrieved
      case FSM_IDLE: {
        // request new image
        if (true == this->is_new_image_requested_()) {
          camera_fb_t* fb = esp_camera_fb_get();
        
          if (fb != nullptr) {
            ESP_LOGD(TAG, "Got Image: len=%u", fb->len);
            this->current_image_ = std::make_shared<CameraImage>(fb, this->single_requesters_ | this->stream_requesters_);
            this->new_image_callback_.call(this->current_image_);
            this->last_update_ = millis();
            this->single_requesters_ = 0U;
          }
          else {
            ESP_LOGW(TAG, "Got invalid frame from camera!");
          }
        }
      } break;
      // 1: We can return the FB to camera
      case FSM_RELEASE_PICTURE: {
        ESP_LOGV(TAG, "Release Image");
        // return framebuffer
        auto *fb = this->current_image_->get_raw_buffer();
        esp_camera_fb_return(fb);
        // Reset shared_ptr
        this->current_image_.reset();
      } break;
      // 2+: Image is still in use
      default: {
        ESP_LOGVV(TAG, "Image still in use");
        // Do nothing
      } break;
  }
}

float ESP32Camera::get_setup_priority() const { return setup_priority::DATA; }

/* ---------------- constructors ---------------- */
ESP32Camera::ESP32Camera(const std::string &name) : EntityBase(name) {
  this->config_.pin_pwdn = -1;
  this->config_.pin_reset = -1;
  this->config_.pin_xclk = -1;
  this->config_.ledc_timer = LEDC_TIMER_0;
  this->config_.ledc_channel = LEDC_CHANNEL_0;
  this->config_.pixel_format = PIXFORMAT_JPEG;
  this->config_.frame_size = FRAMESIZE_VGA;  // 640x480
  this->config_.jpeg_quality = 12;
  this->config_.fb_count = 2;
  //this->config_.grab_mode = CAMERA_GRAB_WHEN_EMPTY  //CAMERA_GRAB_LATEST. Sets when buffers should be filled

  global_esp32_camera = this;
}

ESP32Camera::ESP32Camera() : ESP32Camera("") {}

/* ---------------- setters ---------------- */
/* set pin assignment */
void ESP32Camera::set_data_pins(std::array<uint8_t, 8> pins) {
  this->config_.pin_d0 = pins[0];
  this->config_.pin_d1 = pins[1];
  this->config_.pin_d2 = pins[2];
  this->config_.pin_d3 = pins[3];
  this->config_.pin_d4 = pins[4];
  this->config_.pin_d5 = pins[5];
  this->config_.pin_d6 = pins[6];
  this->config_.pin_d7 = pins[7];
}

void ESP32Camera::set_vsync_pin(uint8_t pin) { this->config_.pin_vsync = pin; }
void ESP32Camera::set_href_pin(uint8_t pin) { this->config_.pin_href = pin; }
void ESP32Camera::set_pixel_clock_pin(uint8_t pin) { this->config_.pin_pclk = pin; }
void ESP32Camera::set_external_clock(uint8_t pin, uint32_t frequency) {
  this->config_.pin_xclk = pin;
  this->config_.xclk_freq_hz = frequency;
}
void ESP32Camera::set_i2c_pins(uint8_t sda, uint8_t scl) {
  this->config_.pin_sscb_sda = sda;
  this->config_.pin_sscb_scl = scl;
}
void ESP32Camera::set_reset_pin(uint8_t pin) { this->config_.pin_reset = pin; }
void ESP32Camera::set_power_down_pin(uint8_t pin) { this->config_.pin_pwdn = pin; }

/* set image parameters */
void ESP32Camera::set_frame_size(ESP32CameraFrameSize size) {
  switch (size) {
    case ESP32_CAMERA_SIZE_160X120:
      this->config_.frame_size = FRAMESIZE_QQVGA;
      break;
    case ESP32_CAMERA_SIZE_176X144:
      this->config_.frame_size = FRAMESIZE_QCIF;
      break;
    case ESP32_CAMERA_SIZE_240X176:
      this->config_.frame_size = FRAMESIZE_HQVGA;
      break;
    case ESP32_CAMERA_SIZE_320X240:
      this->config_.frame_size = FRAMESIZE_QVGA;
      break;
    case ESP32_CAMERA_SIZE_400X296:
      this->config_.frame_size = FRAMESIZE_CIF;
      break;
    case ESP32_CAMERA_SIZE_640X480:
      this->config_.frame_size = FRAMESIZE_VGA;
      break;
    case ESP32_CAMERA_SIZE_800X600:
      this->config_.frame_size = FRAMESIZE_SVGA;
      break;
    case ESP32_CAMERA_SIZE_1024X768:
      this->config_.frame_size = FRAMESIZE_XGA;
      break;
    case ESP32_CAMERA_SIZE_1280X1024:
      this->config_.frame_size = FRAMESIZE_SXGA;
      break;
    case ESP32_CAMERA_SIZE_1600X1200:
      this->config_.frame_size = FRAMESIZE_UXGA;
      break;
  }
}
void ESP32Camera::set_jpeg_quality(uint8_t quality) { this->config_.jpeg_quality = quality; }
void ESP32Camera::set_vertical_flip(bool vertical_flip) { this->vertical_flip_ = vertical_flip; }
void ESP32Camera::set_horizontal_mirror(bool horizontal_mirror) { this->horizontal_mirror_ = horizontal_mirror; }
void ESP32Camera::set_contrast(int contrast) { this->contrast_ = contrast; }
void ESP32Camera::set_brightness(int brightness) { this->brightness_ = brightness; }
void ESP32Camera::set_saturation(int saturation) { this->saturation_ = saturation; }
void ESP32Camera::set_special_effect(ESP32SpecialEffect effect) { this->special_effect_ = effect; }
/* set exposure parameters */
void ESP32Camera::set_aec_mode(ESP32GainControlMode mode) { this->aec_mode_ = mode; }
void ESP32Camera::set_aec2(bool aec2) { this->aec2_ = aec2; }
void ESP32Camera::set_ae_level(int ae_level) { this->ae_level_ = ae_level; }
void ESP32Camera::set_aec_value(uint32_t aec_value) { this->aec_value_ = aec_value; }
/* set gains parameters */
void ESP32Camera::set_agc_mode(ESP32GainControlMode mode) { this->agc_mode_ = mode; }
void ESP32Camera::set_agc_value(uint8_t agc_value) { this->agc_value_ = agc_value; }
void ESP32Camera::set_agc_gain_ceiling(ESP32AgcGainCeiling gain_ceiling) { this->agc_gain_ceiling_ = gain_ceiling; }
/* set white balance */
void ESP32Camera::set_wb_mode(ESP32WhiteBalanceMode mode) { this->wb_mode_ = mode; }
/* set test mode */
void ESP32Camera::set_test_pattern(bool test_pattern) { this->test_pattern_ = test_pattern; }
/* set fps */
void ESP32Camera::set_max_update_interval(uint32_t max_update_interval) {
  this->max_update_interval_ = max_update_interval;
}
void ESP32Camera::set_idle_update_interval(uint32_t idle_update_interval) {
  this->idle_update_interval_ = idle_update_interval;
}

/* ---------------- public API (specific) ---------------- */
void ESP32Camera::add_image_callback(std::function<void(std::shared_ptr<CameraImage>)> &&f) {
  this->new_image_callback_.add(std::move(f));
}
void ESP32Camera::add_stream_start_callback(std::function<void()> &&callback) {
  this->stream_start_callback_.add(std::move(callback));
}
void ESP32Camera::add_stream_stop_callback(std::function<void()> &&callback) {
  this->stream_stop_callback_.add(std::move(callback));
}
void ESP32Camera::start_stream(CameraRequester requester) {
  this->stream_start_callback_.call();
  this->stream_requesters_ |= (1U << requester);
}
void ESP32Camera::stop_stream(CameraRequester requester) {
  this->stream_stop_callback_.call();
  this->stream_requesters_ &= ~(1U << requester);
}
void ESP32Camera::request_image(CameraRequester requester) { this->single_requesters_ |= (1U << requester); }
void ESP32Camera::update_camera_parameters() {
  sensor_t *s = esp_camera_sensor_get();
  /* update image */
  s->set_vflip(s, this->vertical_flip_);
  s->set_hmirror(s, this->horizontal_mirror_);
  s->set_contrast(s, this->contrast_);
  s->set_brightness(s, this->brightness_);
  s->set_saturation(s, this->saturation_);
  s->set_special_effect(s, (int) this->special_effect_);  // 0 to 6
  /* update exposure */
  s->set_exposure_ctrl(s, (bool) this->aec_mode_);
  s->set_aec2(s, this->aec2_);            // 0 = disable , 1 = enable
  s->set_ae_level(s, this->ae_level_);    // -2 to 2
  s->set_aec_value(s, this->aec_value_);  // 0 to 1200
  /* update gains */
  s->set_gain_ctrl(s, (bool) this->agc_mode_);
  s->set_agc_gain(s, (int) this->agc_value_);  // 0 to 30
  s->set_gainceiling(s, (gainceiling_t) this->agc_gain_ceiling_);
  /* update white balance mode */
  s->set_wb_mode(s, (int) this->wb_mode_);  // 0 to 4
  /* update test patern */
  s->set_colorbar(s, this->test_pattern_);
}

/* ---------------- Internal methods ---------------- */
bool ESP32Camera::is_new_image_requested_() {
  bool state_return = false;
  uint32_t update_time_delta = millis() - this->last_update_;
  uint8_t idle_requester_mask = 1U << CameraRequester::IDLE;
  
  // Test single image request condition (except IDLE)
  if (0U != (this->single_requesters_ & ~idle_requester_mask)) {
    // Single request
    ESP_LOGV(TAG, "Image request (Single)");
    state_return = true;
  }
  // Test stream request condition (except IDLE)
  else if ((0U != (this->stream_requesters_ & ~idle_requester_mask)) && (update_time_delta > this->max_update_interval_)) {
    // Stream request
      ESP_LOGV(TAG, "Image request (Normal fps)");
      state_return = true;
  }
  // Test IDLE state condition
  else if (update_time_delta > this->idle_update_interval_) {
    // IDLE request
    ESP_LOGV(TAG, "Image request (IDLE fps)");
    this->single_requesters_ |= idle_requester_mask;
    state_return = true;
  }
  else {
    // Frame not requested --> Do nothing
    state_return = false;
  }

  return state_return;
}

ESP32Camera *global_esp32_camera;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

/* ---------------- CameraImageReader class ---------------- */
void CameraImageReader::set_image(std::shared_ptr<CameraImage> image) {
  this->image_ = std::move(image);
  this->offset_ = 0;
}

size_t CameraImageReader::available() const {
  size_t value_returned = 0U;
  
  if (nullptr != this->image_) {
    if (this->offset_ < this->image_->get_data_length()) {
      value_returned = this->image_->get_data_length() - this->offset_;
    }
  }
  
  return value_returned;
}

void CameraImageReader::return_image() {
  if (this->offset_ != this->image_->get_data_length()) {
    ESP_LOGW(TAG, "Image not completely consumed");
  }
  this->image_.reset();
}

void CameraImageReader::consume_data(size_t consumed) {
  if (this->offset_ + consumed <= this->image_->get_data_length()) {
    this->offset_ += consumed;
  }
  else {
    ESP_LOGW(TAG, "Offset saturated to max len (too many consumed data)");
    this->offset_ = this->image_->get_data_length();
  }
}

uint8_t *CameraImageReader::peek_data_buffer(size_t len) {
  uint8_t* pointer_returned = nullptr;
  
  if ((this->offset_ + len) <= this->image_->get_data_length()) {
    pointer_returned = this->image_->get_data_buffer() + this->offset_;
  }
  else {
    ESP_LOGW(TAG, "Offset inconsistent during transmit of image");
  }

  return pointer_returned;
}

/* ---------------- CameraImage class ---------------- */
CameraImage::CameraImage(camera_fb_t *buffer, uint8_t requesters) : buffer_(buffer), requesters_(requesters) {}

camera_fb_t *CameraImage::get_raw_buffer() { return this->buffer_; }
uint8_t *CameraImage::get_data_buffer() { return this->buffer_->buf; }
size_t CameraImage::get_data_length() { return this->buffer_->len; }
bool CameraImage::was_requested_by(CameraRequester requester) const {
  return (this->requesters_ & (1 << requester)) != 0;
}

}  // namespace esp32_camera
}  // namespace esphome

#endif
