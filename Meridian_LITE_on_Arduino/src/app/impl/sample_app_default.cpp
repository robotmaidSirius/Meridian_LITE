/**
 * @file sample_app_default.cpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-28
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#include "../sample_app_default.hpp"

#include <Arduino.h>
#include <board/pins/meridian_board_lite_pins.hpp>

void SampleAppDefault::loop(Meridim90 &a_meridim90) {
}
void SampleAppDefault::setup() {
  this->servo_left.set_meridim90_start_index(0, 15);
  this->servo_right.set_meridim90_start_index(15, 15);
}

SampleAppDefault::SampleAppDefault() {
  this->servo_left.setup(&Serial1, PINS_DEFAULT_SERVO_1_EN, APP_SERVO_BAUDRATE_L, APP_SERVO_BAUDRATE_L);
  this->servo_right.setup(&Serial2, PINS_DEFAULT_SERVO_2_EN, APP_SERVO_BAUDRATE_R, APP_SERVO_TIMEOUT_R);
}
