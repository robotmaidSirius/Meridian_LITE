/**
 * @file i_mrd_plugin_gpio_inout.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-19
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_PLUGIN_GPIO_HPP
#define I_MRD_PLUGIN_GPIO_HPP

#include "i_mrd_plugin.hpp"

template <typename TYPE>
class I_Meridian_GPIO_InOut : public I_Meridian_Plugin {
public:
  I_Meridian_GPIO_InOut(uint8_t pin, bool is_output = true) {
    this->m_pin = pin;
    this->m_is_output = is_output;
  }

  virtual bool write(TYPE value) = 0;
  virtual TYPE read() = 0;

public:
  bool is_output() { return this->m_is_output; };

protected:
  uint8_t m_pin = 0xFF;
  bool m_is_output = true;
};

#endif // I_MRD_PLUGIN_GPIO_HPP
