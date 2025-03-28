/**
 * @file i_mrd_plugin_eeprom.hpp
 * @brief MeridianCoreで使用するEEPROMのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_PLUGIN_EEPROM_HPP
#define I_MRD_PLUGIN_EEPROM_HPP

#include "i_mrd_plugin.hpp"

class I_Meridian_EEPROM : public I_Meridian_Plugin {
public:
  virtual bool write(uint16_t address, uint8_t data) = 0;
  virtual uint8_t read(uint16_t address) = 0;
};

#endif // I_MRD_PLUGIN_EEPROM_HPP
