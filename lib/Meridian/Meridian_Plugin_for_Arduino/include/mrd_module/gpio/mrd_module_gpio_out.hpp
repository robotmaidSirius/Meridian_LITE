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

class MrdGpioOut : public IMeridianGPIOInOut<int> {
public:
  MrdGpioOut(uint8_t pin, uint8_t index, uint8_t pos = 0) : IMeridianGPIOInOut(pin) {
    assert(0 <= index && index < 10);
    this->m_index = index + (pos / 8);
    this->m_pos = 1 << (pos % 8);
  }
  ~MrdGpioOut() {
    // Do nothing
  }

public:
  bool setup() override {
    if (0xFF != this->m_pin) {
      pinMode(this->m_pin, OUTPUT);
      return true;
    }
    return false;
  }
  bool write(int value) override {
    this->m_flag = (0 < value) ? true : false;
    return true;
  }
  int read() override {
    return this->m_flag ? 1 : 0;
  }

  bool input(Meridim90 &a_meridim) override {
    // Do nothing
    return true;
  }

  bool output(Meridim90 &a_meridim) override {
    digitalWrite(this->m_pin, this->m_flag ? HIGH : LOW);
    if (true == this->m_flag) {
      a_meridim.user_data[this->m_index] = this->m_pos | a_meridim.user_data[this->m_index];
    } else {
      a_meridim.user_data[this->m_index] = ~(this->m_pos) & a_meridim.user_data[this->m_index];
    }
    return true;
  }

private:
  int m_index;
  int m_pos;
  bool m_flag = false;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_GPIO_OUT_HPP__
