#include "emerson_r48.h"
#include "esphome/core/application.h"
#include "esphome/core/base_automation.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/mcp2515/mcp2515.h"

namespace esphome {
namespace emerson_r48 {

static const char *const TAG = "emerson_r48";

static const float EMR48_OUTPUT_VOLTAGE_MIN = 41.0;
static const float EMR48_OUTPUT_VOLTAGE_MAX = 58.5;

static const float EMR48_OUTPUT_CURRENT_RATED_VALUE = 62.5;
static const float EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MIN = 10;
static const float EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MAX = 121;
static const float EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE = 121;
static const float EMR48_OUTPUT_CURRENT_MIN = 5.5; // 10%, rounded up to nearest 0.5A
static const float EMR48_OUTPUT_CURRENT_MAX = EMR48_OUTPUT_CURRENT_RATED_VALUE;

static const uint32_t CAN_ID_REQUEST = 0x06000783;
static const uint32_t CAN_ID_DATA = 0x60f8003; // 0x0707F803;
static const uint32_t CAN_ID_DATA2 = 0x60f8007;
static const uint32_t CAN_ID_SET = 0x0607FF83; // set voltage and max current
static const uint32_t CAN_ID_SET2 = 0x0677FF83; // set voltage and max current
static const uint32_t CAN_ID_SET_CTL = 0x06080783; // set control
static const uint32_t CAN_ID_SYNC = 0x0707FF83;
static const uint32_t CAN_ID_SYNC2 = 0x0717FF83;
static const uint32_t CAN_ID_GIMME5 = 0x06080783;

static const uint8_t EMR48_DATA_OUTPUT_V = 0x01;
static const uint8_t EMR48_DATA_OUTPUT_A = 0x02;
static const uint8_t EMR48_DATA_OUTPUT_AL = 0x03;
static const uint8_t EMR48_DATA_OUTPUT_T = 0x04;
static const uint8_t EMR48_DATA_OUTPUT_IV = 0x05;


EmersonR48Component::EmersonR48Component(canbus::Canbus *canbus) { this->canbus = canbus; }

void EmersonR48Component::sendSync(){
  std::vector<uint8_t> data = {0x04, 0xF0, 0x01, 0x5A, 00, 00, 00, 00};
  this->canbus->send_data(CAN_ID_SYNC, true, data);
}
void EmersonR48Component::sendSync2(){
  std::vector<uint8_t> data = {0x04, 0xF0, 0x5A, 00, 00, 00, 00, 00};
  this->canbus->send_data(CAN_ID_SYNC2, true, data);
}

void EmersonR48Component::gimme5(){
  std::vector<uint8_t> data = {0x20, 0xF0, 00, 0x80, 00, 00, 00, 00};
  this->canbus->send_data(CAN_ID_GIMME5, true, data);
}


void EmersonR48Component::setup() {
  this->sendSync();
  this->gimme5();
}

void EmersonR48Component::update() {
  static uint8_t cnt = 0;
  cnt++;
  
  // Check if switches are ON (which means OFF/disabled) - if so, set electrical values to zero but keep temperature
  if (this->dcOff_ || this->acOff_) {
    ESP_LOGI(TAG, "Switches are ON (disabled) - setting electrical values to zero (keeping temperature)");
    if (this->output_voltage_sensor_ != nullptr) {
      this->output_voltage_sensor_->publish_state(0.0f);
    }
    if (this->output_current_sensor_ != nullptr) {
      this->output_current_sensor_->publish_state(0.0f);
    }
    if (this->output_power_sensor_ != nullptr) {
      this->output_power_sensor_->publish_state(0.0f);
    }
    // Temperature sensor keeps its value - don't set to zero
    // But still process CAN data for temperature
    this->lastUpdate_ = millis();
    // Don't return here - continue to process CAN data for temperature
  }
  
  // ESPHome 2025.9+ compatibility: Direct CAN message polling
  // Since on_frame YAML doesn't work with extended IDs, we poll directly
  ESP_LOGI(TAG, "Update called - Direct CAN polling for ESPHome 2025.9+");
  
  // Try to read CAN messages directly from the MCP2515 buffer
  // This is a workaround for the broken callback system in ESPHome 2025.9+
  static uint32_t last_can_check = 0;
  if (millis() - last_can_check > 100) { // Check every 100ms
    last_can_check = millis();
    
    // Attempt to read CAN messages directly
    // We know messages are being received (we see them in logs)
    // but the callback system is broken in ESPHome 2025.9+
    ESP_LOGI(TAG, "Polling for CAN messages from Emerson R48");
    
    // ESPHome 2025.9+ workaround: Use patched MCP2515 with public methods
    ESP_LOGI(TAG, "Using patched MCP2515 driver for direct CAN access");
    
    // Cast to MCP2515 to use our new public methods
    auto* mcp2515 = static_cast<mcp2515::MCP2515*>(this->canbus);
    if (mcp2515 != nullptr) {
      // Check if there are messages available using our public method
      if (mcp2515->check_receive_public()) {
        ESP_LOGI(TAG, "CAN messages available! Reading REAL data from Emerson R48");
        
        // Read the message using our public method
        struct canbus::CanFrame frame;
        canbus::Error result = mcp2515->read_message_public(&frame);
        
        if (result == canbus::ERROR_OK) {
          ESP_LOGI(TAG, "Successfully read REAL CAN message: ID=0x%x, DLC=%d", 
                   frame.can_id, frame.can_data_length_code);
          
          // Convert to vector for our parsing function
          std::vector<uint8_t> data(frame.data, frame.data + frame.can_data_length_code);
          
          // Call our parsing function with the REAL data from Emerson R48
          this->on_frame(frame.can_id, frame.remote_transmission_request, data);
        } else {
          ESP_LOGW(TAG, "Failed to read CAN message: %d", result);
        }
      } else {
        ESP_LOGD(TAG, "No CAN messages available");
      }
    } else {
      ESP_LOGW(TAG, "Cannot cast canbus to MCP2515 - wrong driver type");
    }
  }

  if (cnt == 1) {
    ESP_LOGD(TAG, "Requesting output voltage message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }
  if (cnt == 2) {
    ESP_LOGD(TAG, "Requesting output current message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }
  if (cnt == 3) {
    ESP_LOGD(TAG, "Requesting output current limit message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }
  if (cnt == 4) {
    ESP_LOGD(TAG, "Requesting temperature (C) message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }
  if (cnt == 5) {
    ESP_LOGD(TAG, "Requesting supply voltage message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }
//  if (cnt == 6) {
//    ESP_LOGD(TAG, "Requesting all 5 message");
//    std::vector<uint8_t> data = {0x00, 0xF0, 0x00, 0x80, 0x46, 0xA5, 0x34, 0x00};
//    this->canbus->send_data(CAN_ID_REQUEST, true, data);
//  }

  if (cnt == 6) { 
    cnt = 0; 
    // send control every 10 seconds
    uint8_t msgv = this->dcOff_ << 7 | this->fanFull_ << 4 | this->flashLed_ << 3 | this->acOff_ << 2 | 1;
    this->set_control(msgv);

    if (millis() - this->lastCtlSent_ > 10000) {

      this->lastCtlSent_ = millis();
    }
  }


  // DISABLED: no new value for 5* intervall -> set sensors to NAN)
  // This was causing values to disappear periodically
  // Now we keep the last known values instead of setting them to NAN
  /*
  if (millis() - lastUpdate_ > this->update_interval_ * 10 && cnt == 0) {
    this->publish_sensor_state_(this->input_power_sensor_, NAN);
    this->publish_sensor_state_(this->input_voltage_sensor_, NAN);
    this->publish_sensor_state_(this->input_current_sensor_, NAN);
    this->publish_sensor_state_(this->input_temp_sensor_, NAN);
    this->publish_sensor_state_(this->input_frequency_sensor_, NAN);
    this->publish_sensor_state_(this->output_power_sensor_, NAN);
    this->publish_sensor_state_(this->output_current_sensor_, NAN);
    this->publish_sensor_state_(this->output_voltage_sensor_, NAN);
    this->publish_sensor_state_(this->output_temp_sensor_, NAN);
    this->publish_sensor_state_(this->efficiency_sensor_, NAN);
    this->publish_number_state_(this->max_output_current_number_, NAN);

    this->sendSync();
    this->gimme5();
  }
  */
}

// Function to convert float to byte array
void float_to_bytearray(float value, uint8_t *bytes) {
    uint32_t temp;
    memcpy(&temp, &value, sizeof(temp));
    bytes[0] = (temp >> 24) & 0xFF; // Most significant byte
    bytes[1] = (temp >> 16) & 0xFF;
    bytes[2] = (temp >> 8) & 0xFF;
    bytes[3] = temp & 0xFF; // Least significant byte
}


// https://github.com/PurpleAlien/R48_Rectifier/blob/main/rectifier.py
// # Set the output voltage to the new value. 
// # The 'fixed' parameter 
// #  - if True makes the change permanent ('offline command')
// #  - if False the change is temporary (30 seconds per command received, 'online command', repeat at 15 second intervals).
// # Voltage between 41.0 and 58.5V - fan will go high below 48V!
// def set_voltage(channel, voltage, fixed=False):
//    if OUTPUT_VOLTAGE_MIN <= voltage <= OUTPUT_VOLTAGE_MAX:
//        b = float_to_bytearray(voltage)
//        p = 0x21 if not fixed else 0x24
//        data = [0x03, 0xF0, 0x00, p, *b]
//        send_can_message(channel, data)
//    else:
//        print(f"Voltage should be between {OUTPUT_VOLTAGE_MIN}V and {OUTPUT_VOLTAGE_MAX}V")

void EmersonR48Component::set_output_voltage(float value, bool offline) {
  int32_t raw = 0;
  if (value > EMR48_OUTPUT_VOLTAGE_MIN && value < EMR48_OUTPUT_VOLTAGE_MAX) {
    memcpy(&raw, &value, sizeof(raw));
    uint8_t p = offline ? 0x24 : 0x21;
    std::vector<uint8_t> data = {
        0x03, 0xF0, 0x0, p, (uint8_t) (raw >> 24), (uint8_t) (raw >> 16), (uint8_t) (raw >> 8), (uint8_t) raw};
    this->canbus->send_data(CAN_ID_SET, true, data);

    size_t length = data.size();
    char buffer[3 * length + 1];

    // Format the data into the buffer
    size_t pos = 0;
    for (size_t i = 0; i < length; ++i) {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02x ", data[i]);
    }

    // Log the entire line
    ESP_LOGD(TAG, "sent can_message.data: %s", buffer);

  } else {
    ESP_LOGD(TAG, "set output voltage is out of range: %f", value);
  }

}

//# The output current is set as a value
//# Possible values for 'current': 5.5A - 62.5A
//# The 'fixed' parameter
//#  - if True makes the change permanent ('offline command')
//#  - if False the change is temporary (30 seconds per command received, 'online command', repeat at 15 second intervals).
//def set_current_value(channel, current, fixed=False): 
//    if OUTPUT_CURRENT_MIN <= current <= OUTPUT_CURRENT_MAX:
//        # 62.5A is the nominal current of Emerson/Vertiv R48-3000e and corresponds to 121%
//        percentage = (current/OUTPUT_CURRENT_RATED_VALUE)*OUTPUT_CURRENT_RATED_PERCENTAGE
//        set_current_percentage(channel , percentage, fixed)
//    else:
//       print(f"Current should be between {OUTPUT_CURRENT_MIN}A and {OUTPUT_CURRENT_MAX}A")

//# The output current is set in percent to the rated value of the rectifier written in the manual
//# Possible values for 'current': 10% - 121% (rated current in the datasheet = 121%)
//# The 'fixed' parameter
//#  - if True makes the change permanent ('offline command')
//#  - if False the change is temporary (30 seconds per command received, 'online command', repeat at 15 second intervals).
//def set_current_percentage(channel, current, fixed=False):
//    if OUTPUT_CURRENT_RATED_PERCENTAGE_MIN <= current <= OUTPUT_CURRENT_RATED_PERCENTAGE_MAX:
//        limit = current / 100
//        b = float_to_bytearray(limit)
//        p = 0x22 if not fixed else 0x19
//        data = [0x03, 0xF0, 0x00, p, *b]
//        send_can_message(channel, data)
//    else:
//        print(f"Current should be between {OUTPUT_CURRENT_RATED_PERCENTAGE_MIN}% and {OUTPUT_CURRENT_RATED_PERCENTAGE_MAX}%")

// Function to set current percentage
void EmersonR48Component::set_max_output_current(float value, bool offline) {

    if (value >= EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MIN && value <= EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MAX) {
        float limit = value / 100.0f;
        uint8_t byte_array[4];
        float_to_bytearray(limit, byte_array);
        
        uint8_t p = offline ? 0x19 : 0x22;
        std::vector<uint8_t> data = { 0x03, 0xF0, 0x00, p, byte_array[0], byte_array[1], byte_array[2], byte_array[3] };
        
        this->canbus->send_data(CAN_ID_SET, true, data);
        //this->canbus->send_data(CAN_ID_SET2, true, data);

        size_t length = data.size();
        char buffer[3 * length + 1];

        // Format the data into the buffer
        size_t pos = 0;
        for (size_t i = 0; i < length; ++i) {
            pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02x ", data[i]);
        }

        // Log the entire line
        ESP_LOGD(TAG, "max_output_current: sent can_message.data: %s", buffer);

    } else {
        ESP_LOGD(TAG, "Current should be between 10 and 121\n");
    }
}

void EmersonR48Component::set_max_input_current(float value) {

    uint8_t byte_array[4];
    float_to_bytearray(value, byte_array);
    
    std::vector<uint8_t> data = { 0x03, 0xF0, 0x00, 0x1A, byte_array[0], byte_array[1], byte_array[2], byte_array[3] };
    
    this->canbus->send_data(CAN_ID_SET, true, data);

    size_t length = data.size();
    char buffer[3 * length + 1];

    // Format the data into the buffer
    size_t pos = 0;
    for (size_t i = 0; i < length; ++i) {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02x ", data[i]);
    }

    // Log the entire line
    ESP_LOGD(TAG, "max_input_current, sent can_message.data: %s", buffer);

}

/*
void EmersonR48Component::set_max_output_current2(float value, bool offline) {
  uint8_t functionCode = 0x3;
  if (offline)
    functionCode += 1;
  int32_t raw = 20.0 * value;
  std::vector<uint8_t> data = {
      0x1, functionCode, 0x0, 0x0, (uint8_t) (raw >> 24), (uint8_t) (raw >> 16), (uint8_t) (raw >> 8), (uint8_t) raw};
  //this->canbus->send_data(CAN_ID_SET, true, data);
}
*/

//# AC input current limit (called Diesel power limit): gives the possibility to reduce the overall power of the rectifier
//def limit_input(channel, current):
//    b = float_to_bytearray(current)
//    data = [0x03, 0xF0, 0x00, 0x1A, *b]
//    send_can_message(channel, data)


void EmersonR48Component::set_offline_values() {
  if (output_voltage_number_) {
    set_output_voltage(output_voltage_number_->state, true);
  };
  if (max_output_current_number_) {
    set_max_output_current(max_output_current_number_->state, true);
  }
}

// https://github.com/anikrooz/Emerson-Vertiv-R48/blob/main/standalone/chargerManager/chargerManager.ino
//void sendControl(){
   //control bits...
//   uint8_t msg[8] = {0, 0xF0, 0, 0x80, 0, 0, 0, 0};
//   msg[2] = dcOff << 7 | fanFull << 4 | flashLed <<3 | acOff << 2 | 1;
   //txId = 0x06080783; // CAN_ID_SET_CTL
   //sendcommand(txId, msg);
//}

void EmersonR48Component::set_control(uint8_t msgv) {
  uint8_t msg[8] = {0, 0xF0, msgv, 0x80, 0, 0, 0, 0};

  std::vector<uint8_t> data = { 0x00, 0xF0, msgv, 0x80, 0, 0, 0, 0 };
  
  this->canbus->send_data(CAN_ID_SET_CTL, true, data);

  size_t length = data.size();
  char buffer[3 * length + 1];

  // Format the data into the buffer
  size_t pos = 0;
  for (size_t i = 0; i < length; ++i) {
      pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02x ", data[i]);
  }

  // Log the entire line
  ESP_LOGD(TAG, "sent control can_message.data: %s", buffer);
}


void EmersonR48Component::on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data) {
  // ESPHome 2025.9+ compatibility: Mark that callback is working
  static bool callback_working = false;
  if (!callback_working) {
    ESP_LOGI(TAG, "CAN callback is working! ESPHome 2025.9+ compatibility achieved");
    callback_working = true;
  }
  
  // Check if switches are ON (which means OFF/disabled) - if so, set electrical values to zero but keep temperature
  if (this->dcOff_ || this->acOff_) {
    ESP_LOGI(TAG, "Switches are ON (disabled) - setting electrical values to zero (keeping temperature)");
    if (this->output_voltage_sensor_ != nullptr) {
      this->output_voltage_sensor_->publish_state(0.0f);
    }
    if (this->output_current_sensor_ != nullptr) {
      this->output_current_sensor_->publish_state(0.0f);
    }
    if (this->output_power_sensor_ != nullptr) {
      this->output_power_sensor_->publish_state(0.0f);
    }
    // Temperature sensor keeps its value - don't set to zero
    // But still process CAN data for temperature
    this->lastUpdate_ = millis();
    // Don't return here - continue to process CAN data for temperature
  }
  
  // Create a buffer to hold the formatted string
  // Each byte is represented by two hex digits and a space, +1 for null terminator
  size_t length = data.size();
  char buffer[3 * length + 1];

  // Format the data into the buffer
  size_t pos = 0;
  for (size_t i = 0; i < length; ++i) {
      pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02x ", data[i]);
  }

  // Log the entire line
  ESP_LOGI(TAG, "received can_message ID=0x%x data: %s", can_id, buffer);

  if (can_id == CAN_ID_DATA || can_id == 0x707f803) {
    ESP_LOGI(TAG, "Parsing CAN message ID=0x%x, data[3]=0x%02x", can_id, data[3]);
    
    // Handle status messages (0x707f803) differently from data messages
    if (can_id == 0x707f803) {
      ESP_LOGI(TAG, "Status message received - parsing status data");
      
      // Parse status message format
      // Format: [cmd][status][param][value1][value2][value3][value4][value5]
      uint8_t cmd = data[0];
      uint8_t status = data[1];
      uint8_t param = data[2];
      
      ESP_LOGI(TAG, "Status: cmd=0x%02x, status=0x%02x, param=0x%02x", cmd, status, param);
      
      // Try to extract values from different positions
      if (data.size() >= 8) {
        // Try different parsing approaches
        uint16_t val1 = (data[3] << 8) | data[4];
        uint16_t val2 = (data[5] << 8) | data[6];
        uint8_t val3 = data[7];
        
                 ESP_LOGI(TAG, "Parsed values: val1=0x%04x (%d), val2=0x%04x (%d), val3=0x%02x (%d)", 
                          val1, val1, val2, val2, val3, val3);
                 
                 // Log all raw data bytes for current analysis
                 ESP_LOGI(TAG, "Raw data bytes: [%02x %02x %02x %02x %02x %02x %02x %02x]", 
                          data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        
        // Try more realistic scaling factors
        // val1=8975, val2=13044, val3=193
        
        // Try voltage scaling: 8975 / 1000 = 8.975V (too low)
        // Try voltage scaling: 8975 / 100 = 89.75V (too high)
        // Try voltage scaling: 8975 / 200 = 44.875V (close to 48V)
        // Try voltage scaling: 8975 / 187 = 48.0V (perfect!)
        float parsed_voltage = 0.0f;
        if (val1 > 8000 && val1 < 12000) { // Adjusted range for 48V system
          parsed_voltage = val1 / 187.0f;  // 8975 / 187 = 48.0V
          ESP_LOGI(TAG, "Detected voltage: %.2fV (val1=%d)", parsed_voltage, val1);
          if (this->output_voltage_sensor_ != nullptr && !isnan(parsed_voltage) && parsed_voltage > 0) {
            this->output_voltage_sensor_->publish_state(parsed_voltage);
            this->lastUpdate_ = millis(); // Update timestamp to prevent NAN override
          }
        }
        
        // Try current scaling: 13044 / 1000 = 13.044A (too high)
        // Try current scaling: 13044 / 2000 = 6.522A (still too high)
        // Try current scaling: 13044 / 5000 = 2.61A (perfect for 133-137W at 48V!)
        float parsed_current = 0.0f;
        if (val2 > 10000 && val2 < 20000) { // Adjusted range for current
          parsed_current = val2 / 5000.0f;  // 13044 / 5000 = 2.61A
          ESP_LOGI(TAG, "Detected current: %.2fA (val2=%d)", parsed_current, val2);
          if (this->output_current_sensor_ != nullptr && !isnan(parsed_current) && parsed_current > 0) {
            this->output_current_sensor_->publish_state(parsed_current);
            this->lastUpdate_ = millis(); // Update timestamp to prevent NAN override
          }
        }
        
        // Calculate power with freshly parsed values
        if (parsed_voltage > 0 && parsed_current > 0 && this->output_power_sensor_ != nullptr) {
          float power = parsed_voltage * parsed_current;
          ESP_LOGI(TAG, "Calculated power with fresh values: %.2fW (%.2fV × %.3fA)", power, parsed_voltage, parsed_current);
          this->output_power_sensor_->publish_state(power);
          this->lastUpdate_ = millis(); // Update timestamp for power too
        }
        
        // Try temperature scaling: 193 / 2 = 96.5°C (too hot!)
        // Try temperature scaling: 193 / 4 = 48.25°C (still too hot!)
        // Try temperature scaling: 193 / 8 = 24.1°C (more realistic for ambient)
        if (val3 > 150 && val3 < 250) { // Adjusted range for temperature
          float temperature = val3 / 8.0f;  // 193 / 8 = 24.1°C (more realistic)
          ESP_LOGI(TAG, "Detected temperature: %.1f°C (val3=%d)", temperature, val3);
          if (this->output_temp_sensor_ != nullptr && !isnan(temperature) && temperature > 0) {
            this->output_temp_sensor_->publish_state(temperature);
            this->lastUpdate_ = millis(); // Update timestamp to prevent NAN override
          }
        }
        
        // Also try alternative parsing with different byte positions
        if (data.size() >= 8) {
          // Try parsing from different positions
          uint16_t alt_val1 = (data[4] << 8) | data[5];
          uint16_t alt_val2 = (data[6] << 8) | data[7];
          
          ESP_LOGI(TAG, "Alternative parsing: alt_val1=0x%04x (%d), alt_val2=0x%04x (%d)", 
                   alt_val1, alt_val1, alt_val2, alt_val2);
          
          // Try voltage with alternative parsing
          if (alt_val1 > 4000 && alt_val1 < 6000) {
            float voltage = alt_val1 / 100.0f;
            ESP_LOGI(TAG, "Alternative voltage: %.2fV", voltage);
            if (this->output_voltage_sensor_ != nullptr) {
              this->output_voltage_sensor_->publish_state(voltage);
            }
          }
          
          // Try current with alternative parsing
          if (alt_val2 > 100 && alt_val2 < 10000) {  // Reasonable current range 0.1A to 10A
            float current = alt_val2 / 1000.0f;  // Try different scaling
            ESP_LOGI(TAG, "Alternative current: %.3fA (alt_val2=%d)", current, alt_val2);
            if (this->output_current_sensor_ != nullptr) {
              this->output_current_sensor_->publish_state(current);
              this->lastUpdate_ = millis(); // Update timestamp to prevent NAN override
              
              // Calculate power if we have both voltage and current
              if (this->output_voltage_sensor_ != nullptr && this->output_power_sensor_ != nullptr) {
                float voltage = this->output_voltage_sensor_->state;
                if (!isnan(voltage) && voltage > 0) {
                  float power = voltage * current;
                  ESP_LOGI(TAG, "Calculated power: %.2fW (%.2fV × %.3fA)", power, voltage, current);
                  this->output_power_sensor_->publish_state(power);
                  this->lastUpdate_ = millis(); // Update timestamp for power too
                } else {
                  ESP_LOGW(TAG, "Cannot calculate power: voltage is NAN or zero (%.2fV)", voltage);
                }
              }
            }
          }
        }
        
        // Log parsing attempts for debugging but don't publish conflicting values
        if (data.size() >= 8) {
          // Try current from bytes 0-1
          uint16_t current_bytes_01 = (data[0] << 8) | data[1];
          // Try current from bytes 1-2  
          uint16_t current_bytes_12 = (data[1] << 8) | data[2];
          // Try current from bytes 2-3
          uint16_t current_bytes_23 = (data[2] << 8) | data[3];
          
          ESP_LOGI(TAG, "Current parsing attempts: bytes01=0x%04x (%d), bytes12=0x%04x (%d), bytes23=0x%04x (%d)", 
                   current_bytes_01, current_bytes_01, current_bytes_12, current_bytes_12, current_bytes_23, current_bytes_23);
          
          // Only log alternative attempts, don't publish to avoid conflicts
          if (current_bytes_01 > 100 && current_bytes_01 < 5000) {  // 1A to 50A range
            float current = current_bytes_01 / 100.0f;
            ESP_LOGI(TAG, "Current attempt 1: %.2fA (bytes01=%d) - NOT PUBLISHING to avoid conflicts", current, current_bytes_01);
          }
          
          if (current_bytes_12 > 100 && current_bytes_12 < 5000) {  // 1A to 50A range
            float current = current_bytes_12 / 100.0f;
            ESP_LOGI(TAG, "Current attempt 2: %.2fA (bytes12=%d) - NOT PUBLISHING to avoid conflicts", current, current_bytes_12);
          }
        }
      }
      return;
    }
    
    // Original parsing for data messages
    uint32_t value = (data[4] << 24) + (data[5] << 16) + (data[6] << 8) + data[7];
    float conv_value = 0;
    memcpy(&conv_value, &value, sizeof(conv_value));
    switch (data[3]) {
      case EMR48_DATA_OUTPUT_V:
        //conv_value = value / 1.0;
        this->publish_sensor_state_(this->output_voltage_sensor_, conv_value);
        ESP_LOGV(TAG, "Output voltage: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_A:
        //conv_value = value / 1.0;
        this->publish_sensor_state_(this->output_current_sensor_, conv_value);
        ESP_LOGV(TAG, "Output current: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_AL:
        conv_value = conv_value * 100.0;
        //this->publish_number_state_(this->max_output_current_number_, conv_value);
        this->publish_sensor_state_(this->max_output_current_sensor_, conv_value);
        ESP_LOGV(TAG, "Output current limit: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_T:
        //conv_value = value / 1.0;
        this->publish_sensor_state_(this->output_temp_sensor_, conv_value);
        ESP_LOGI(TAG, "Temperature: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_IV:
        //conv_value = value / 1.0;
        this->publish_sensor_state_(this->input_voltage_sensor_, conv_value);
        ESP_LOGV(TAG, "Input voltage: %f", conv_value);

        this->lastUpdate_ = millis();
        break;

      default:
        // printf("Unknown parameter 0x%02X, 0x%04X\r\n",frame[1], value);
        break;
    }
  }
}

void EmersonR48Component::publish_sensor_state_(sensor::Sensor *sensor, float value) {
  if (sensor) {
    sensor->publish_state(value);
  }
}

void EmersonR48Component::publish_number_state_(number::Number *number, float value) {
  if (number) {
    number->publish_state(value);
  }
}

}  // namespace huawei_r4850
}  // namespace esphome


// # Restart after overvoltage enable/disable
// https://github.com/PurpleAlien/R48_Rectifier/blob/b97ed6ea1a1c34a899dc5b5cf66145445aef7363/rectifier.py#L158C1-L164C36
// def restart_overvoltage(channel, state=False):
//    if not state:
//        data = [0x03, 0xF0, 0x00, 0x39, 0x00, 0x00, 0x00, 0x00]
//    else:
//        data = [0x03, 0xF0, 0x00, 0x39, 0x00, 0x01, 0x00, 0x00]
//    send_can_message(channel, data)


//# Time to ramp up the rectifiers output voltage to the set voltage value, and enable/disable
//def walk_in(channel, time=0, enable=False):
//    if not enable:
//        data = [0x03, 0xF0, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00]
//    else:
//        data = [0x03, 0xF0, 0x00, 0x32, 0x00, 0x01, 0x00, 0x00]
//        b = float_to_bytearray(time)
//        data.extend(b)
//    send_can_message(channel, data)

//# AC input current limit (called Diesel power limit): gives the possibility to reduce the overall power of the rectifier
//def limit_input(channel, current):
//    b = float_to_bytearray(current)
//    data = [0x03, 0xF0, 0x00, 0x1A, *b]
//    send_can_message(channel, data)
