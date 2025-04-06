/**
 * @file meridian_board_lite_pins.hpp
 * @brief "meridian Board -LITE-"のピン番号を定義しています
 * @version 1.2.0
 * @date 2025-01-24
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MERIDIAN_BOARD_LITE_PINS_HPP__
#define __MERIDIAN_BOARD_LITE_PINS_HPP__

////////////////////////////////////////////////////
// PINS
////////////////////////////////////////////////////
// SERIAL
#define PINS_DEFAULT_SERIAL0_RX (3) /**< Serial0の受信IO番号 */
#define PINS_DEFAULT_SERIAL0_TX (1) /**< Serial0の送信IO番号 */

#define PINS_DEFAULT_SERIAL1_RX (27) /**< Serial1の受信IO番号 */
#define PINS_DEFAULT_SERIAL1_TX (32) /**< Serial1の送信IO番号 */

#define PINS_DEFAULT_SERIAL2_RX (16) /**< Serial2の受信IO番号 */
#define PINS_DEFAULT_SERIAL2_TX (17) /**< Serial2の送信IO番号 */

// GPIO
#define PINS_DEFAULT_ANALOG_IN_1 (34) /**< analog1のPin番号 */
#define PINS_DEFAULT_ANALOG_IN_2 (35) /**< analog2のPin番号 */
#define PINS_DEFAULT_GPIO_1      (25) /**< DAC1のPin番号 */
#define PINS_DEFAULT_GPIO_2      (26) /**< DAC2のPin番号 */

// SERVO
#define PINS_DEFAULT_SERVO_1_EN (33)                    /**< ICR_Lの有効化ピン */
#define PINS_DEFAULT_SERVO_1_TX PINS_DEFAULT_SERIAL1_TX /**< ICR_Lの送信ピン */
#define PINS_DEFAULT_SERVO_1_RX PINS_DEFAULT_SERIAL1_RX /**< ICR_Lの受信ピン */

#define PINS_DEFAULT_SERVO_2_EN (4)                     /**< ICR_Rの有効化ピン */
#define PINS_DEFAULT_SERVO_2_TX PINS_DEFAULT_SERIAL2_TX /**< ICR_Rの送信ピン */
#define PINS_DEFAULT_SERVO_2_RX PINS_DEFAULT_SERIAL2_RX /**< ICR_Rの受信ピン */

// I2C
#ifndef BOARD_PRINT_I2C
#define PINS_DEFAULT_I2C_SCL (22) /**< I2C SCL */
#define PINS_DEFAULT_I2C_SDA (21) /**< I2C SDA */

#ifndef BOARD_SUPPRESSES_WARNINGS
#warning "[JP] ESP32-DevKitCの標準設定です。ボードに記載されているI2C-SCL/SDAの表記の反対になってます"
#warning "[EN] This is the standard setting for the ESP32-DevKitC. The I2C-SCL/SDA notation on the board is the opposite."
#endif
#else
#define PINS_DEFAULT_I2C_SCL (21) /**< I2C SCL */
#define PINS_DEFAULT_I2C_SDA (22) /**< I2C SDA */
#endif

// SPI
#define PINS_DEFAULT_SPI_MISO (19) /**< SPI MISO */
#define PINS_DEFAULT_SPI_MOSI (23) /**< SPI MOSI */
#define PINS_DEFAULT_SPI_SCK  (18) /**< SPI SCK */
#define PINS_DEFAULT_SPI_CS1  (5)  /**< SPI CS1 */
#define PINS_DEFAULT_SPI_CS2  (15) /**< SPI CS2 */

#endif
