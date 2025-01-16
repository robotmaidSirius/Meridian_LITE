/**
 * @file i_mrd_sd.hpp
 * @brief
 * @version 0.25.1
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_SD_HPP
#define I_MRD_SD_HPP

#include <stdint.h>

class I_Meridian_SD {
public:
  virtual ~I_Meridian_SD() = default;
  virtual void write(uint16_t address, uint8_t data) = 0;
  virtual uint8_t read(uint16_t address) = 0;
};

#endif // I_MRD_SD_HPP
