/**
 * @file fireflake_base_board_for_pico_w.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-21
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef FIREFLAKE_BASE_BOARD_FOR_PICO_W_HPP
#define FIREFLAKE_BASE_BOARD_FOR_PICO_W_HPP

#include "pico/stdlib.h"

////////////////////////////////////////////////////
// PINS
////////////////////////////////////////////////////
// GPIO - OUTPUT
//     CN1: 1-GP6, 2-NC, 3-GP7, 4-NC
#define PINS_GP_OUT1 (6)
#define PINS_GP_OUT2 (7)
//     CN1: 5-GP20, 6-NC, 7-GP21, 8-NC
// GPIO - INPUT
#define PINS_GP_IN1 (20)
#define PINS_GP_IN2 (21)
// I2C
//     CN2: 1-SCL, 2-SDA, 3-VCC, 4-GND
//     CN3: 1-SCL, 2-SDA, 3-VCC, 4-GND
#define PINS_I2C_SCL (5)
#define PINS_I2C_SDA (4)
// SPI
//     CN5: 1-VCC, 2-MISO, 3-CS, 4-SCK, 5-MOSI, 6-GND
#define PINS_SPI_MISO (16)
#define PINS_SPI_CS   (17)
#define PINS_SPI_SCK  (18)
#define PINS_SPI_MOSI (19)
// ADC
//     CN6: 1-ADC0, 2-ADC1, 3-A_GND
#define PINS_ADC0 (26)
#define PINS_ADC1 (27)

// RS232C: Need to connect the JP3/JP5 jumpers.
//     CN8: 1-RxD, 2-TxD, 3-VCC, 4-GND
// RS422/485: Need to connect the JP4/JP6 jumpers.the termination is JP7.
//     CN11: 1-A, 2-B, 3-NC, 4-GND
#define PINS_UART_TX (0)
#define PINS_UART_RX (1)

#define PINS_RS485_EN (22)

// Extended Connector A
//     CN7: 1-I2C_SCL, 2-I2C_SDA, 3-GND
//          4-UART_TX, 5-UART_RX, 6-GND
//          7-SPI_CLK, 8-SPI_MOSI, 9-SPI_MISO, 10-SPI_CS
#define PINS_DAUGHTER_I2C_SCL (3)
#define PINS_DAUGHTER_I2C_SDA (2)

#define PINS_DAUGHTER_UART_TX (8)
#define PINS_DAUGHTER_UART_RX (9)

#define PINS_DAUGHTER_SPI_CLK  (10)
#define PINS_DAUGHTER_SPI_MOSI (11)
#define PINS_DAUGHTER_SPI_MISO (12)
#define PINS_DAUGHTER_SPI_CS   (13)

// Extended Connector B
//     CN8: 1-VCC5V, 2-VCC3v3, 3-GND
//          4-GP14, 5-GP15, 6-GP22, 7-GND
//          8-ADC1, 9-ADC2, 10-A_GND
#define PINS_DAUGHTER_GP1      (14)
#define PINS_DAUGHTER_GP2      (15)
#define PINS_DAUGHTER_RS485_EN (22)

#define PINS_DAUGHTER_ADC1 (27)
#define PINS_DAUGHTER_ADC2 (28)

////////////////////////////////////////////////////
// RS485 Control
////////////////////////////////////////////////////
#define RS485_CNTRL_SEND     HIGH
#define RS485_CNTRL_RECEIVED LOW

////////////////////////////////////////////////////
// Other
////////////////////////////////////////////////////
#define PINS_BOARD_LED (25)

#endif // FIREFLAKE_BASE_BOARD_FOR_PICO_W_HPP
