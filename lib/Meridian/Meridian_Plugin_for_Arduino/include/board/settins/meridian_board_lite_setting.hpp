/**
 * @file meridian_board_lite_settings.hpp
 * @brief "meridian Board -LITE-"のデフォルト設定を定義しています
 * @version 1.2.0
 * @date 2025-01-24
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MERIDIAN_BOARD_LITE_SETTINGS_HPP__
#define __MERIDIAN_BOARD_LITE_SETTINGS_HPP__

/// @brief プラグインの名前
#define PLUGIN_NAME "Meridian_Plugin_for_Arduino"
/// @brief ボードの名前
#define PLUGIN_BOARD_NAME "Meridian Board -LITE-"
/// @brief プラグインのバージョン
#define PLUGIN_VERSION "v1.2.0"

////////////////////////////////////////////////////
// SETTINGS
////////////////////////////////////////////////////
#ifndef BOARD_SETTING_MOUNTED_BOOT_STANDBY
/// @brief 起動時の待機時間[ms]
/// @details ボード搭載のコンデンサの充電時間として待機時間です。
#define BOARD_SETTING_MOUNTED_BOOT_STANDBY (200)
#endif

#ifndef BOARD_SETTING_DEFAULT_INTERVAL_MS
/// @brief デフォルトの周期[ms]
#define BOARD_SETTING_DEFAULT_INTERVAL_MS (10)
#endif

#ifndef BOARD_SETTING_DEFAULT_I2C_SPEED
/// @brief デフォルトのI2Cのスピード
#define BOARD_SETTING_DEFAULT_I2C_SPEED (400000UL)
#endif

#ifndef BOARD_SETTING_DEFAULT_I2C_MAX
/// @brief 接続するI2Cポートの最大数
/// @details I2Cポートに接続するデバイスを増やす場合は、この値を増やしてください。デフォルト値は(4)です。
#define BOARD_SETTING_DEFAULT_I2C_MAX (4)
#endif

#ifndef BOARD_SETTING_DEFAULT_SERIAL0_BAUD
/// @brief Serial0のデフォルトボーレート
#define BOARD_SETTING_DEFAULT_SERIAL0_BAUD (115200)
#endif

#ifndef BOARD_SETTING_DEFAULT_SERIAL0_CONFIG
/// @brief Serial0のデフォルト設定
#define BOARD_SETTING_DEFAULT_SERIAL0_CONFIG (SERIAL_8N1)
#endif

#ifndef BOARD_SETTING_DEFAULT_SERIAL1_BAUD
/// @brief Serial1のデフォルトボーレート
#define BOARD_SETTING_DEFAULT_SERIAL1_BAUD (1250000)
#endif

#ifndef BOARD_SETTING_DEFAULT_SERIAL1_CONFIG
/// @brief Serial1のデフォルト設定
#define BOARD_SETTING_DEFAULT_SERIAL1_CONFIG (SERIAL_8N1)
#endif

#ifndef BOARD_SETTING_DEFAULT_SERIAL2_BAUD
/// @brief Serial2のデフォルトボーレート
#define BOARD_SETTING_DEFAULT_SERIAL2_BAUD (1250000)
#endif

#ifndef BOARD_SETTING_DEFAULT_SERIAL2_CONFIG
/// @brief Serial2のデフォルト設定
#define BOARD_SETTING_DEFAULT_SERIAL2_CONFIG (SERIAL_8N1)
#endif

#ifndef BOARD_SETTING_DEFAULT_SPI_SPEED
/// @brief SPI通信の速度
#define BOARD_SETTING_DEFAULT_SPI_SPEED (6000000)
#endif

#endif // __MERIDIAN_BOARD_LITE_SETTINGS_HPP__
