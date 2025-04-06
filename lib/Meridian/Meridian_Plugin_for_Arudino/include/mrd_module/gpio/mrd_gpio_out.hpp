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

#include <Arduino.h>
#include <mrd_module/mrd_plugin/i_mrd_plugin_gpio_in_out.hpp>

namespace meridian {
namespace modules {
namespace plugin {

class MrdGpioOut : public IMeridianGPIOInOut<bool> {
public:
  MrdGpioOut(uint8_t pin, int index = -1, int pos = -1) : IMeridianGPIOInOut(pin) {
    assert(-1 <= index && index < MERIDIM90_USER_DATA_SIZE);
    assert(-1 <= pos && pos < 8);
    if ((0 <= index && index < MERIDIM90_USER_DATA_SIZE) && (0 <= pos && pos < 8)) {
      this->m_index = index + (pos / 8);
      this->m_pos = 1 << (pos % 8);
      this->m_output = true;
    } else {
      this->m_index = -1;
      this->m_pos = -1;
      this->m_output = false;
    }
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
  bool write(bool value) override {
    this->m_flag = value;
    return true;
  }
  bool write(int value, bool output) {
    this->write(value);
    if (output) {
      digitalWrite(this->m_pin, this->m_flag ? HIGH : LOW);
    }
    return true;
  }
  bool read() override {
    return this->m_flag;
  }

  bool input(Meridim90 &a_meridim) override {
    // Do nothing
    return true;
  }

  bool output(Meridim90 &a_meridim) override {
    digitalWrite(this->m_pin, this->m_flag ? HIGH : LOW);
    if (this->m_output) {
      if (this->m_flag) {
        a_meridim.userdata.options[this->m_index] = this->m_pos | a_meridim.userdata.options[this->m_index];
      } else {
        a_meridim.userdata.options[this->m_index] = ~(this->m_pos) & a_meridim.userdata.options[this->m_index];
      }
    }
    return true;
  }

private:
  int m_index;
  int m_pos;
  bool m_flag = false;
  bool m_output = false;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_GPIO_OUT_HPP__
