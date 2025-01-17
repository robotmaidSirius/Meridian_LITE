/**
 * @file i_mrd_i2c.hpp
 * @brief MeridianCoreで使用するI2Cのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_I2C_HPP
#define I_MRD_I2C_HPP

#include <stdint.h>

class I_Meridian_I2C {
public:
  virtual ~I_Meridian_I2C() = default;
  virtual void write(uint8_t address, uint8_t data) = 0;
  virtual uint8_t read(uint8_t address) = 0;
};

#endif // I_MRD_I2C_HPP
