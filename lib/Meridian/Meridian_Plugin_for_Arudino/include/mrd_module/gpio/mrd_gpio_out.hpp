/**
 * @file mrd_gpio_out.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-20
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_GPIO_OUT_HPP
#define MRD_GPIO_OUT_HPP

// ライブラリ導入
#include "mrd_plugin/i_mrd_plugin_gpio_inout.hpp"
#include <Arduino.h>

class MrdGpioOut : public I_Meridian_GPIO_InOut<int> {
public:
  MrdGpioOut(uint8_t pin) : I_Meridian_GPIO_InOut(pin, true) {}
  ~MrdGpioOut() {}

public:
  bool setup() override {
    if (0xFF != this->m_pin) {
      pinMode(this->m_pin, OUTPUT);
      return true;
    }
  }
  bool write(int value) override {
    digitalWrite(this->m_pin, value == 0 ? LOW : HIGH);
    return true;
  }
  int read() override {
    return digitalRead(this->m_pin);
  }

  bool refresh(Meridim90Union &a_meridim) override {
    return true;
  }
};

#endif // MRD_GPIO_OUT_HPP
