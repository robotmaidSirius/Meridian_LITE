/**
 * @file i_mrd_plugin_eeprom.hpp
 * @brief MeridianCoreで使用するEEPROMのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __I_MRD_PLUGIN_EEPROM_HPP__
#define __I_MRD_PLUGIN_EEPROM_HPP__

#include "i_mrd_plugin.hpp"

namespace meridian {
namespace modules {
namespace plugin {

class IMeridianEEPROM : public IMeridianPlugin {
public:
  virtual bool write(uint16_t address, uint8_t data) = 0;
  virtual uint8_t read(uint16_t address) = 0;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_EEPROM_HPP__
