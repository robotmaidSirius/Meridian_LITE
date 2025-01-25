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

#include "pins/meridian_board_lite_pins.hpp"
#include "settins/meridian_board_lite_setting.hpp"

#include "mrd_modules/mrd_plugin/i_mrd_plugin_gpio_in_out.hpp"
#include "mrd_modules/mrd_plugin/i_mrd_plugin_i2c.hpp"
#include "mrd_modules/mrd_plugin/i_mrd_plugin_servo.hpp"
#include "mrd_modules/mrd_plugin/i_mrd_plugin_spi.hpp"

namespace meridian {
namespace board {
namespace meridian_board_lite {
using namespace meridian::modules::plugin;
const int MERIDIAN_BOARD_LITE_ANALOG_NUM = 2;
const int MERIDIAN_BOARD_LITE_GPIO_NUM = 2;
const int MERIDIAN_BOARD_LITE_I2C_NUM = MERIDIAN_BOARD_LITE_SETTINGS_I2C_MAX;

struct mrd_entity {
public:
  IMeridianGPIOInOut<int> *analog[MERIDIAN_BOARD_LITE_ANALOG_NUM];
  IMeridianGPIOInOut<int> *gpio[MERIDIAN_BOARD_LITE_GPIO_NUM];
  IMeridianI2C *i2c[MERIDIAN_BOARD_LITE_I2C_NUM];
  IMeridianSPI *spi_outside;
  IMeridianSPI *spi_inside;
  IMeridianServo *servo_left;
  IMeridianServo *servo_right;
};

struct mrd_parameters {
public:
  unsigned int interval_ms = 10;
  unsigned int i2c_speed = 400000UL; //! I2Cの速度（400kHz推奨）
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
