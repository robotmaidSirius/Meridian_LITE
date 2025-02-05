/**
 * @file mrd_module_gpio_in.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-19
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_GPIO_IN_HPP__
#define __MRD_MODULE_GPIO_IN_HPP__

// ライブラリ導入
#include <Arduino.h>
#include <mrd_module/mrd_plugin/i_mrd_plugin_gpio_in_out.hpp>

namespace meridian {
namespace modules {
namespace plugin {

#define UNUSED_PARAM(x) // 未使用とするマクロ

class MrdGpioIN : public IMeridianGPIOInOut<int> {

public:
  MrdGpioIN(uint8_t pin, int index, int pos = 0) : IMeridianGPIOInOut(pin) {
    this->m_index = index;
    this->m_pos = 1 << pos;
  }
  ~MrdGpioIN() {
    // Do nothing
  }

public:
  bool setup() override {
    if (0xFF != this->m_pin) {
      pinMode(this->m_pin, INPUT_PULLUP);
      return true;
    }
    return false;
  }
  bool write(int value) override {
    UNUSED_PARAM(value);
    return false;
  }
  int read() override {
    return digitalRead(this->m_pin);
  }

  bool input(Meridim90 &a_meridim) override {
    if (0 < this->read()) {
      a_meridim.userdata.options[this->m_index] = this->m_pos | a_meridim.userdata.options[this->m_index];
    } else {
      a_meridim.userdata.options[this->m_index] = ~(this->m_pos) & a_meridim.userdata.options[this->m_index];
    }
    return true;
  }

  bool output(Meridim90 &a_meridim) override {
    // Do nothing
    return true;
  }

private:
  int m_index;
  int m_pos;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_GPIO_IN_HPP__
