/**
 * @file i_mrd_plugin_i2c.hpp
 * @brief MeridianCoreで使用するI2Cのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_PLUGIN_I2C_HPP
#define I_MRD_PLUGIN_I2C_HPP

#include "i_mrd_plugin.hpp"

class I_Meridian_I2C : public I_Meridian_Plugin {
public:
  virtual void write(uint8_t address, uint8_t data) = 0;
  virtual uint8_t read(uint8_t address) = 0;

protected:
  uint8_t m_id;
};

#endif // I_MRD_PLUGIN_I2C_HPP
