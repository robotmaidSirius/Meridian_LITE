/**
 * @file meridian_board_lite_for_esp32.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MERIDIAN_BOARD_LITE_FOR_ESP32_HPP
#define MERIDIAN_BOARD_LITE_FOR_ESP32_HPP

#include "meridian_core.hpp"
#if 0
#include "mrd_plugin/i_mrd_ahrs.hpp"
#include "mrd_plugin/i_mrd_eeprom.hpp"
#include "mrd_plugin/i_mrd_gpio_in.hpp"
#include "mrd_plugin/i_mrd_gpio_out.hpp"
#include "mrd_plugin/i_mrd_i2c.hpp"
#include "mrd_plugin/i_mrd_joypad.hpp"
#include "mrd_plugin/i_mrd_sd.hpp"
#include "mrd_plugin/i_mrd_servo.hpp"
#include "mrd_plugin/i_mrd_wifi.hpp"
#endif

#define I2C0_SPEED 400000 // I2Cの速度（400kHz推奨）

//================================================================================================================
//  MERIDIAN - LITE - ESP32の配線
//================================================================================================================
//
// ESP32devkitC  -  デバイス
//   3V3         -  BNO005 VIN
//   21          -  BNO005 SCL
//   22          -  BNO005 SDA
//   GND         -  BNO005 GND
//
//   4  EN       -  ICS変換基板 R系統 EN
//   16 RX       -  ICS変換基板 R系統 TX
//   17 TX       -  ICS変換基板 R系統 RX
//   5V          -  ICS変換基板 IOREF
//   GND         -  ICS変換基板 GND
//
//   33 EN       -  ICS変換基板 L系統 EN
//   32 RX       -  ICS変換基板 L系統 TX
//   27 TX       -  ICS変換基板 L系統 RX
//   5V          -  ICS変換基板 IOREF
//   GND         -  ICS変換基板 GND
//
//   27          -  SPI_MISO
//   23          -  SPI_MOSI
//   18          -  SPI_CSK
//   15          -  SPI_CS SD

#endif // MERIDIAN_BOARD_LITE_FOR_ESP32_HPP
