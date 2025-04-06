/**
 * @file i_mrd_plugin_gpio_inout.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-19
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __I_MRD_PLUGIN_GPIO_HPP__
#define __I_MRD_PLUGIN_GPIO_HPP__

#include "i_mrd_plugin.hpp"

namespace meridian {
namespace modules {
namespace plugin {

template <typename TYPE>
class IMeridianGPIOInOut : public IMeridianPlugin {
public:
  IMeridianGPIOInOut(uint8_t pin, bool is_output = true) {
    this->m_pin = pin;
    this->m_is_output = is_output;
  }

public:
  virtual bool write(TYPE value) = 0;
  virtual TYPE read() = 0;

protected:
  uint8_t m_pin = 0xFF;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_GPIO_HPP__
