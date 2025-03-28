/**
 * @file mrd_analog.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-19
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_ANALOG_HPP
#define MRD_ANALOG_HPP

// ライブラリ導入
#include "mrd_plugin/i_mrd_plugin_gpio_inout.hpp"
#include <Arduino.h>

#define UNUSED_PARAM(x) // 未使用とするマクロ

class MrdAnalog : public I_Meridian_GPIO_InOut<int> {
public:
  MrdAnalog(uint8_t pin) : I_Meridian_GPIO_InOut(pin, false) {}
  ~MrdAnalog() {}

public:
  bool setup() override {
    pinMode(this->m_pin, ANALOG);
    return true;
  }
  bool write(int value) override {
    UNUSED_PARAM(value);
    return false;
  }
  int read() override {
    return analogRead(this->m_pin);
  }

  bool refresh(Meridim90Union &a_meridim) override {
    return true;
  }
};

#endif // MRD_ANALOG_HPP
