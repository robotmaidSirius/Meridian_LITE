/**
 * @file mrd_module_gpio_out.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-20
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_GPIO_OUT_HPP__
#define __MRD_MODULE_GPIO_OUT_HPP__

#include "mrd_modules/mrd_plugin/i_mrd_plugin_gpio_in_out.hpp"
#include <Arduino.h>

namespace meridian {
namespace modules {
namespace plugin {

class MrdGpioOut : public I_Meridian_GPIO_InOut<int> {
public:
  MrdGpioOut(uint8_t pin, uint8_t index, uint8_t pos = 0) : I_Meridian_GPIO_InOut(pin, true) {
    assert(0 <= index && index < 10);
    this->m_index = index + (pos / 8);
    this->m_pos = 1 << (pos % 8);
  }
  ~MrdGpioOut() {}

public:
  bool setup() override {
    if (0xFF != this->m_pin) {
      pinMode(this->m_pin, OUTPUT);
      return true;
    }
    return false;
  }
  bool write(int value) override {
    digitalWrite(this->m_pin, value == 0 ? LOW : HIGH);
    return true;
  }
  int read() override {
    return digitalRead(this->m_pin);
  }

  bool refresh(Meridim90 &a_meridim) override {
    if (true == this->is_output()) {
      this->write(a_meridim.user_data[this->m_index] && m_pos);
    } else {
      if (0 < this->read()) {
        a_meridim.user_data[this->m_index] = this->m_pos || a_meridim.user_data[this->m_index];
      } else {
        a_meridim.user_data[this->m_index] = ~(this->m_pos) && a_meridim.user_data[this->m_index];
      }
    }
    return true;
  }

private:
  int m_index;
  int m_pos;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_GPIO_OUT_HPP__
