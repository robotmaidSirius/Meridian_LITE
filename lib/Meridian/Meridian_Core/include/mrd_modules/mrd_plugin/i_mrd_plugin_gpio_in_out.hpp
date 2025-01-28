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

class IMeridianGPIOInOutStatus {
public:
  bool initalized = false;
  bool setup = false;
  bool happened_error = false;

public:
  void all_ok() {
    this->initalized = true;
    this->setup = true;
    this->happened_error = false;
  }
};

template <typename TYPE>
class IMeridianGPIOInOut : public IMeridianPlugin {
public:
  IMeridianGPIOInOut(uint8_t pin) {
    this->m_pin = pin;
  }

public:
  virtual bool write(TYPE value) = 0;
  virtual TYPE read() = 0;

protected:
  uint8_t m_pin = 0xFF;

public:
  void get_status(IMeridianGPIOInOutStatus &state) {
    state.initalized = this->a_state.initalized;
    state.setup = this->a_state.setup;
    state.happened_error = this->a_state.happened_error;
  }

protected:
  IMeridianGPIOInOutStatus a_state;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_GPIO_HPP__
