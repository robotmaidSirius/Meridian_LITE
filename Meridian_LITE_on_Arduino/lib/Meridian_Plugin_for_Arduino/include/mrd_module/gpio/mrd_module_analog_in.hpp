/**
 * @file mrd_module_analog.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-19
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_ANALOG_HPP__
#define __MRD_MODULE_ANALOG_HPP__

// ライブラリ導入
#include "mrd_modules/mrd_plugin/i_mrd_plugin_gpio_in_out.hpp"
#include <Arduino.h>

namespace meridian {
namespace modules {
namespace plugin {

#define UNUSED_PARAM(x) // 未使用とするマクロ

class MrdAnalogIn : public I_Meridian_GPIO_InOut<int> {
public:
  MrdAnalogIn(uint8_t pin, int index) : I_Meridian_GPIO_InOut(pin, false) {
    this->m_index = index;
  }
  ~MrdAnalogIn() {}

public:
  bool setup() override {
    if (0xFF != this->m_pin) {
      pinMode(this->m_pin, ANALOG);
      return true;
    }
    return false;
  }
  bool write(int value) override {
    UNUSED_PARAM(value);
    return false;
  }
  int read() override {
    return analogRead(this->m_pin);
  }

  bool refresh(Meridim90 &a_meridim) override {
    if (true == this->is_output()) {
      this->write(a_meridim.user_data[this->m_index]);
    } else {
      a_meridim.user_data[this->m_index] = this->read();
    }
    return true;
  }

private:
  int m_index;
};

#undef UNUSED_PARAM

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_ANALOG_HPP__
