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

#include "mrd_communication/i_mrd_diagnostic.hpp"
#include <Arduino.h>

namespace meridian {
namespace core {
namespace communication {

class MrdDiagnosticUart : public meridian::core::communication::IMeridianDiagnostic {
public:
  MrdDiagnosticUart(HardwareSerial serial, uint32_t baudrate = 115200) {
    this->_serial = &serial;
    this->_baudrate = baudrate;
  };

  bool setup() override {
    if (nullptr != this->_serial) {
      Serial1.begin(this->_baudrate);
      return true;
    }
    return false;
  };

  size_t message(OUTPUT_LOG_LEVEL level, const char *message) override {
    if (nullptr != this->_serial) {
      return this->_serial->printf("[%s] %s", get_text_level(level), message);
    }
    return 0;
  };

private:
  HardwareSerial *_serial;
  uint32_t _baudrate = 115200;
};

} // namespace communication
} // namespace core
} // namespace meridian

#endif // MRD_DIAGNOSTIC_UART_HPP
