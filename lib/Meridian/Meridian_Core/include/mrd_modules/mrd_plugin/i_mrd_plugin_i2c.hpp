/**
 * @file i_mrd_plugin_i2c.hpp
 * @brief MeridianCoreで使用するI2Cのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __I_MRD_PLUGIN_I2C_HPP__
#define __I_MRD_PLUGIN_I2C_HPP__

#include "i_mrd_plugin.hpp"

namespace meridian {
namespace modules {
namespace plugin {

class IMeridianI2C : public IMeridianPlugin {
public:
  IMeridianI2C(uint8_t address) {
    this->m_address = address;
  }

public:
  virtual void write(uint8_t address, uint8_t data) = 0;
  virtual uint8_t read(uint8_t address) = 0;

protected:
  uint8_t m_address;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_I2C_HPP__
