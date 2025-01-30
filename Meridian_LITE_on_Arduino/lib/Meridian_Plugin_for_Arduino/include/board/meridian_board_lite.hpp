/**
 * @file meridian_board_lite.hpp
 * @brief "meridian Board -LITE-"のための定義をまとめたヘッダファイル
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __BOARD_MERIDIAN_BOARD_LITE_HPP__
#define __BOARD_MERIDIAN_BOARD_LITE_HPP__

#include "meridian_core_for_arduino.hpp"

#include "pins/meridian_board_lite_pins.hpp"
#include "settins/meridian_board_lite_setting.hpp"

#include <mrd_communication/i_mrd_conversation.hpp>
#include <mrd_communication/i_mrd_diagnostic.hpp>
#if BOARD_IN_DETAILED_CLASS
#include <mrd_module/mrd_plugin/i_mrd_plugin_eeprom.hpp>
#include <mrd_module/mrd_plugin/i_mrd_plugin_gpio_in_out.hpp>
#include <mrd_module/mrd_plugin/i_mrd_plugin_i2c.hpp>
#include <mrd_module/mrd_plugin/i_mrd_plugin_pad.hpp>
#include <mrd_module/mrd_plugin/i_mrd_plugin_sd.hpp>
#include <mrd_module/mrd_plugin/i_mrd_plugin_servo.hpp>
#include <mrd_module/mrd_plugin/i_mrd_plugin_spi.hpp>
#else
#include <mrd_module/mrd_plugin/i_mrd_plugin.hpp>
#endif
namespace meridian {
namespace board {
namespace meridian_board_lite {
using namespace meridian::core::communication;
using namespace meridian::modules::plugin;

const int MERIDIAN_BOARD_LITE_ANALOG_NUM = 2;                          ///< アナログ入力の数
const int MERIDIAN_BOARD_LITE_GPIO_NUM = 2;                            ///< GPIOの数
const int MERIDIAN_BOARD_LITE_I2C_NUM = BOARD_SETTING_DEFAULT_I2C_MAX; ///< I2Cに接続するデバイス数

struct mrd_diagnosis {
public:
  /// @brief 通信用のインターフェースと診断用のインターフェースをまとめた構造体
  struct mrd_communicationStatus {
  public:
    IMeridianConversation::Status con; ///< 通信用のインターフェース
    IMeridianDiagnostic::Status diag;  ///< 診断用のインターフェース
  };
  /// @brief プラグインのインターフェースをまとめた構造体
  struct mrd_pluginStatus {
  public:
#if BOARD_IN_DETAILED_CLASS
    IMeridianGPIOInOutStatus analog[MERIDIAN_BOARD_LITE_ANALOG_NUM]; ///< アナログ入力のインターフェース
    IMeridianGPIOInOutStatus gpio[MERIDIAN_BOARD_LITE_GPIO_NUM];     ///< GPIOのインターフェース
    IMeridianI2CStatus i2c[MERIDIAN_BOARD_LITE_I2C_NUM];             ///< I2Cのインターフェース
    IMeridianEEPROMStatus eeprom;                                    ///< EEPROMのインターフェース
    IMeridianSDStatus sd_card;                                       ///< SDカード用のSPIのインターフェース
    IMeridianSPIStatus spi;                                          ///< SPIのインターフェース
    IMeridianServoStatus servo_left;                                 ///< ICS_Lのインターフェース
    IMeridianServoStatus servo_right;                                ///< ICS_Rのインターフェース
    IMeridianPadStatus pad;                                          ///< パッドのインターフェース
#else
    IMeridianPlugin::Status analog[MERIDIAN_BOARD_LITE_ANALOG_NUM]; ///< アナログ入力のインターフェース
    IMeridianPlugin::Status gpio[MERIDIAN_BOARD_LITE_GPIO_NUM];     ///< GPIOのインターフェース
    IMeridianPlugin::Status i2c[MERIDIAN_BOARD_LITE_I2C_NUM];       ///< I2Cのインターフェース
    IMeridianPlugin::Status eeprom;                                 ///< EEPROMのインターフェース
    IMeridianPlugin::Status sd_card;                                ///< SDカード用のSPIのインターフェース
    IMeridianPlugin::Status spi;                                    ///< SPIのインターフェース
    IMeridianPlugin::Status servo_left;                             ///< ICS_Lのインターフェース
    IMeridianPlugin::Status servo_right;                            ///< ICS_Rのインターフェース
    IMeridianPlugin::Status pad;                                    ///< パッドのインターフェース
#endif
  };

public:
  mrd_communicationStatus communication; ///< 通信用のインターフェース
  mrd_pluginStatus plugin;               ///< プラグインのインターフェース
};
extern mrd_diagnosis diagnosis;

struct mrd_entity {
public:
  /// @brief 通信用のインターフェースと診断用のインターフェースをまとめた構造体
  struct mrd_communication {
  public:
    IMeridianConversation *con; ///< 通信用のインターフェース
    IMeridianDiagnostic *diag;  ///< 診断用のインターフェース
  };
  /// @brief プラグインのインターフェースをまとめた構造体
  struct mrd_plugin {
  public:
#if BOARD_IN_DETAILED_CLASS
    IMeridianGPIOInOut<int> *analog[MERIDIAN_BOARD_LITE_ANALOG_NUM]; ///< アナログ入力のインターフェース
    IMeridianGPIOInOut<int> *gpio[MERIDIAN_BOARD_LITE_GPIO_NUM];     ///< GPIOのインターフェース
    IMeridianI2C *i2c[MERIDIAN_BOARD_LITE_I2C_NUM];                  ///< I2Cのインターフェース
    IMeridianEEPROM *eeprom;                                         ///< EEPROMのインターフェース
    IMeridianSD *sd_card;                                            ///< SDカード用のSPIのインターフェース
    IMeridianSPI *spi;                                               ///< SPIのインターフェース
    IMeridianServo *servo_left;                                      ///< ICS_Lのインターフェース
    IMeridianServo *servo_right;                                     ///< ICS_Rのインターフェース
    IMeridianPad *pad;                                               ///< パッドのインターフェース
#else
    IMeridianPlugin *analog[MERIDIAN_BOARD_LITE_ANALOG_NUM]; ///< アナログ入力のインターフェース
    IMeridianPlugin *gpio[MERIDIAN_BOARD_LITE_GPIO_NUM];     ///< GPIOのインターフェース
    IMeridianPlugin *i2c[MERIDIAN_BOARD_LITE_I2C_NUM];       ///< I2Cのインターフェース
    IMeridianPlugin *eeprom;                                 ///< EEPROMのインターフェース
    IMeridianPlugin *sd_card;                                ///< SDカード用のSPIのインターフェース
    IMeridianPlugin *spi;                                    ///< SPIのインターフェース
    IMeridianPlugin *servo_left;                             ///< ICS_Lのインターフェース
    IMeridianPlugin *servo_right;                            ///< ICS_Rのインターフェース
    IMeridianPlugin *pad;                                    ///< パッドのインターフェース
#endif
  };

public:
  mrd_communication communication; ///< 通信用のインターフェース
  mrd_plugin plugin;               ///< プラグインのインターフェース
};

/// @brief "meridian Board -LITE-"の動作パラメータ
struct mrd_parameters {
  unsigned int interval_ms = BOARD_SETTING_DEFAULT_INTERVAL_MS; ///< メインループのインターバル (10ms推奨)
  uint32_t i2c_speed = BOARD_SETTING_DEFAULT_I2C_SPEED;         ///< I2Cの速度 (400kHz推奨)
  uint32_t ics_l_speed = BOARD_SETTING_DEFAULT_SERIAL1_BAUD;    ///< ICS_Lの速度 (1.25MHz推奨)
  uint32_t ics_r_speed = BOARD_SETTING_DEFAULT_SERIAL2_BAUD;    ///< ICS_Lの速度 (1.25MHz推奨)
  uint32_t spi_speed = BOARD_SETTING_DEFAULT_SPI_SPEED;         ///< SPIの速度 (6000000kHz推奨)
};

bool board_setup(mrd_entity *a_entity, mrd_parameters *a_param);
Meridim90 mrd_input();
bool mrd_output(Meridim90 &a_meridim90);

} // namespace meridian_board_lite
} // namespace board
} // namespace meridian

using namespace meridian::board::meridian_board_lite;

#endif
