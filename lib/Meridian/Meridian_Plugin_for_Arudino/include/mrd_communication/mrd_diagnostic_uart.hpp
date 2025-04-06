/**
 * @file mrd_diagnostic_uart.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-24
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_DIAGNOSTIC_UART_HPP
#define MRD_DIAGNOSTIC_UART_HPP

#include <Arduino.h>
#include <mrd_communication/i_mrd_diagnostic.hpp>

namespace meridian {
namespace core {
namespace communication {

class MrdDiagnosticUart : public meridian::core::communication::IMeridianDiagnostic {
public:
  MrdDiagnosticUart(HardwareSerial *serial, uint32_t baudrate = 115200, OUTPUT_LOG_LEVEL level = OUTPUT_LOG_LEVEL::LEVEL_INFO) {
    this->disable();
    this->_serial = serial;
    this->_baudrate = baudrate;
    this->set_level(level);
  }

public:
  const char *get_name() override { return "UART"; }
  bool setup() override {
    if (nullptr != this->_serial) {
      Serial1.begin(this->_baudrate);
      this->enable();
      return true;
    }
    return false;
  }

  size_t message(OUTPUT_LOG_LEVEL level, const char *message) override {
    if (nullptr != this->_serial) {
      if (OUTPUT_LOG_LEVEL::LEVEL_OPERATIONAL == level) {
        return this->_serial->printf("%s", message);
      } else {
        return this->_serial->printf("[%s] %s\n", this->get_text_level(level), message);
      }
    }
    return 0;
  }

private:
  HardwareSerial *_serial;
  uint32_t _baudrate = 115200;
};

} // namespace communication
} // namespace core
} // namespace meridian

#endif // MRD_DIAGNOSTIC_UART_HPP
