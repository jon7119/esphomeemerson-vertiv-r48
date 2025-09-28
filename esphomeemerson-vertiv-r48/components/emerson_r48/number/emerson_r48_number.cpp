#include "emerson_r48_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace emerson_r48 {

static const int8_t SET_VOLTAGE_FUNCTION = 0x0;
static const int8_t SET_CURRENT_FUNCTION = 0x3;
static const int8_t SET_INPUT_CURRENT_FUNCTION = 0x4;

void EmersonR48Number::control(float value) {
  switch (this->functionCode_) {
    case SET_VOLTAGE_FUNCTION:
      parent_->set_output_voltage(value);
      this->publish_state(value);
      break;
    case SET_CURRENT_FUNCTION:
      parent_->set_max_output_current(value);
      this->publish_state(value);
      break;
    case SET_INPUT_CURRENT_FUNCTION:
      parent_->set_max_input_current(value);
      this->publish_state(value);
      break;

    default:
      break;
  }
}

}  // namespace emerson_r48
}  // namespace esphome
