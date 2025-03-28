/**
 * @file mrd_gpio_in.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-19
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_GPIO_IN_HPP
#define MRD_GPIO_IN_HPP

// ライブラリ導入
#include "mrd_plugin/i_mrd_plugin_gpio_inout.hpp"
#include <Arduino.h>

#define UNUSED_PARAM(x) // 未使用とするマクロ

class MrdGpioIN : public I_Meridian_GPIO_InOut<int> {

public:
  MrdGpioIN(uint8_t pin) : I_Meridian_GPIO_InOut(pin, false) {}
  ~MrdGpioIN() {}

public:
  bool setup() override {
    if (0xFF != this->m_pin) {
      pinMode(this->m_pin, INPUT_PULLDOWN);
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

  bool refresh(Meridim90Union &a_meridim) override {
    return true;
  }
};

#endif // MRD_GPIO_IN_HPP
