/**
 * @file meridian_board_lite_pins.hpp
 * @brief
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
// GPIO
#define PINS_DEFAULT_ANALOG_IN_1 (34)
#define PINS_DEFAULT_ANALOG_IN_2 (35)
#define PINS_DEFAULT_GPIO_1      (25)
#define PINS_DEFAULT_GPIO_2      (26)

// SERVO
#define PINS_DEFAULT_SERVO_1_EN (33)
#define PINS_DEFAULT_SERVO_1_TX (32)
#define PINS_DEFAULT_SERVO_1_RX (27)

#define PINS_DEFAULT_SERVO_2_EN (4)
#define PINS_DEFAULT_SERVO_2_TX (16)
#define PINS_DEFAULT_SERVO_2_RX (17)

// I2C
#ifndef BOARD_PRINT_I2C
#define PINS_DEFAULT_I2C_SCL (22)
#define PINS_DEFAULT_I2C_SDA (21)
#warning "[JP] ESP32-DevKitCの標準設定です。ボードに記載されているI2C-SCL/SDAの表記の反対になってます"
#warning "[EN] This is the standard setting for the ESP32-DevKitC. The I2C-SCL/SDA notation on the board is the opposite."
#else
#define PINS_DEFAULT_I2C_SCL (21)
#define PINS_DEFAULT_I2C_SDA (22)
#endif

// SPI
#define PINS_DEFAULT_SPI_MISO (19)
#define PINS_DEFAULT_SPI_MOSI (23)
#define PINS_DEFAULT_SPI_SCK  (18)
#define PINS_DEFAULT_SPI_CS1  (5)
#define PINS_DEFAULT_SPI_CS2  (15)

#endif
