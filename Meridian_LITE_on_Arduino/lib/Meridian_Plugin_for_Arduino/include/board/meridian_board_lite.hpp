/**
 * @file meridian_board_lite.hpp
 * @brief Meridian Board Liteのための定義をまとめたヘッダファイル
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __BOARD_MERIDIAN_BOARD_LITE_HPP__
#define __BOARD_MERIDIAN_BOARD_LITE_HPP__

#include "meridian_core.hpp"
using namespace meridian::core::meridim;

#include "mrd_modules/mrd_plugin/i_mrd_plugin_gpio_in_out.hpp"
#include "mrd_modules/mrd_plugin/i_mrd_plugin_i2c.hpp"

////////////////////////////////////////////////////
// SETTINGS
////////////////////////////////////////////////////
#ifndef MERIDIAN_BOARD_LITE_SETTINGS_I2C_MAX
#define MERIDIAN_BOARD_LITE_SETTINGS_I2C_MAX (4)
#endif

////////////////////////////////////////////////////
// PINS
////////////////////////////////////////////////////
// GPIO
#define PINS_ANALOG_IN_1 (34)
#define PINS_ANALOG_IN_2 (35)
#define PINS_GPIO_1      (25)
#define PINS_GPIO_2      (26)

// SERVO
#define PINS_SERVO_1_EN (33)
#define PINS_SERVO_1_TX (32)
#define PINS_SERVO_1_RX (27)

#define PINS_SERVO_2_EN (4)
#define PINS_SERVO_2_TX (16)
#define PINS_SERVO_2_RX (17)

// I2C
#define PINS_I2C_SCL (22)
#define PINS_I2C_SDA (21)

// SPI
#define PINS_SPI_MISO (19)
#define PINS_SPI_MOSI (23)
#define PINS_SPI_SCK  (18)
#define PINS_SPI_CS1  (15)
#define PINS_SPI_CS2  (5)

////////////////////////////////////////////////////
// Other
////////////////////////////////////////////////////

////////////////////////////////////////////////////
////////////////////////////////////////////////////

namespace meridian {
namespace board {
namespace meridian_board_lite {
using namespace meridian::modules::plugin;
const int MERIDIAN_BOARD_LITE_GPIO_NUM = 4;
const int MERIDIAN_BOARD_LITE_I2C_NUM = MERIDIAN_BOARD_LITE_SETTINGS_I2C_MAX;

struct mrd_entity {
public:
  I_Meridian_GPIO_InOut<int> *gpio[MERIDIAN_BOARD_LITE_GPIO_NUM];
  I_Meridian_I2C *i2c[MERIDIAN_BOARD_LITE_I2C_NUM] = {nullptr};
};

struct mrd_parameters {
public:
  unsigned int interval_ms;
  unsigned int i2c_speed = 400000; //! I2Cの速度（400kHz推奨）
};

bool mrd_setup(mrd_entity *a_entity, mrd_parameters *a_param);

Meridim90 mrd_input();
bool mrd_control(Meridim90 &mrd_meridim);
bool mrd_output(Meridim90 &mrd_meridim);

} // namespace meridian_board_lite
} // namespace board
} // namespace meridian

using namespace meridian::board::meridian_board_lite;

#endif
