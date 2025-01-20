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
#include "mrd_plugin/i_mrd_plugin_ahrs.hpp"
#include "mrd_plugin/i_mrd_plugin_eeprom.hpp"
#include "mrd_plugin/i_mrd_plugin_gpio_inout.hpp"
#include "mrd_plugin/i_mrd_plugin_i2c.hpp"
#include "mrd_plugin/i_mrd_plugin_joypad.hpp"
#include "mrd_plugin/i_mrd_plugin_sd.hpp"
#include "mrd_plugin/i_mrd_plugin_servo.hpp"
#include "mrd_plugin/i_mrd_plugin_wifi.hpp"
#include <Wire.h>

#ifndef MERIDIAN_BOARD_LITE_FOR_ESP32_CONFIG_I2C_NUM
#define MERIDIAN_BOARD_LITE_FOR_ESP32_CONFIG_I2C_NUM 10
#endif

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

struct meridian_board_lite_for_esp32_config {

public:
  I_Meridian_AHRS<float, float, float> *ahrs = nullptr;
  I_Meridian_EEPROM *eeprom = nullptr;
  I_Meridian_GPIO_InOut<int> *analog_1 = nullptr;
  I_Meridian_GPIO_InOut<int> *analog_2 = nullptr;
  I_Meridian_GPIO_InOut<int> *gpio_1 = nullptr;
  I_Meridian_GPIO_InOut<int> *gpio_2 = nullptr;
  I_Meridian_I2C *i2c[MERIDIAN_BOARD_LITE_FOR_ESP32_CONFIG_I2C_NUM] = {nullptr};
  I_Meridian_Joypad *joypad = nullptr;
  I_Meridian_SD *sd_card = nullptr;
  I_Meridian_Servo *servo_l = nullptr;
  I_Meridian_Servo *servo_r = nullptr;
  I_Meridian_Wifi *wifi = nullptr;
};

/// @brief Wire0 I2C通信を初期化し, 指定されたクロック速度で設定する.
/// @param a_i2c0_speed I2C通信のクロック速度です.
/// @param a_pinSDA SDAのピン番号. 下記と合わせて省略可.
/// @param a_pinSCL SCLのピン番号. 上記と合わせて省略可.
bool mrd_wire0_init(int a_i2c0_speed, int a_pinSDA = -1, int a_pinSCL = -1) {
  if (a_pinSDA == -1 && a_pinSCL == -1) {
    Wire.begin();
  } else {
    Wire.begin(a_pinSDA, a_pinSCL);
  }
  Wire.setClock(a_i2c0_speed);
  return true;
}

bool meridian_board_lite_for_esp32_setup(meridian_board_lite_for_esp32_config &config, unsigned int interval_ms = 10);

bool meridian_board_lite_for_esp32_input(Meridim90Union &mrd_meridim);
bool meridian_board_lite_for_esp32_control(Meridim90Union &mrd_meridim);
bool meridian_board_lite_for_esp32_output(Meridim90Union &mrd_meridim);

#endif // MERIDIAN_BOARD_LITE_FOR_ESP32_HPP
