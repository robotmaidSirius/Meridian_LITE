/**
 * @file meridian_board_lite_for_esp32.cpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-19
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#include "board/meridian_board_lite_for_esp32.hpp"
meridian_board_lite_for_esp32_config *mrs_config = nullptr;

unsigned int mrs_interval_ms;

bool setup_meridian_board_lite_for_esp32(meridian_board_lite_for_esp32_config &config, unsigned int interval_ms) {
  bool result = true;
  mrs_config = &config;
  mrs_interval_ms = interval_ms;
  //
  if (nullptr != mrs_config->ahrs) {
    result &= mrs_config->ahrs->setup();
  }
  if (nullptr != mrs_config->eeprom) {
    result &= mrs_config->eeprom->setup();
  }
  if (nullptr != mrs_config->analog_1) {
    result &= mrs_config->gpio_1->setup();
  }
  if (nullptr != mrs_config->gpio_2) {
    result &= mrs_config->gpio_2->setup();
  }
  if (nullptr != mrs_config->analog_1) {
    result &= mrs_config->analog_1->setup();
  }
  if (nullptr != mrs_config->analog_2) {
    result &= mrs_config->analog_2->setup();
  }
  for (int i = 0; i < MERIDIAN_BOARD_LITE_FOR_ESP32_CONFIG_I2C_NUM; i++) {
    if (nullptr != mrs_config->i2c[i]) {
      result &= mrs_config->i2c[i]->setup();
    }
  }
  if (nullptr != mrs_config->joypad) {
    result &= mrs_config->joypad->setup();
  }
  if (nullptr != mrs_config->sd_card) {
    result &= mrs_config->sd_card->setup();
  }
  if (nullptr != mrs_config->servo_l) {
    result &= mrs_config->servo_l->setup();
  }
  if (nullptr != mrs_config->servo_r) {
    result &= mrs_config->servo_r->setup();
  }
  if (nullptr != mrs_config->wifi) {
    result &= mrs_config->wifi->setup();
  }

  return result;
}

bool input_meridian_board_lite_for_esp32(Meridim90Union &mrd_meridim) {
  // Input
  if (nullptr != mrs_config->wifi) {
    mrs_config->wifi->received(mrd_meridim);
  }
  if (nullptr != mrs_config->ahrs) {
    mrs_config->ahrs->refresh(mrd_meridim);
  }
  if (nullptr != mrs_config->joypad) {
    mrs_config->joypad->refresh(mrd_meridim);
  }
  if (nullptr != mrs_config->analog_1) {
    mrs_config->analog_1->refresh(mrd_meridim);
  }
  if (nullptr != mrs_config->analog_2) {
    mrs_config->analog_2->refresh(mrd_meridim);
  }
  if (nullptr != mrs_config->gpio_1) {
    if (false == mrs_config->gpio_1->is_output()) {
      mrs_config->gpio_1->refresh(mrd_meridim);
    }
  }
  if (nullptr != mrs_config->gpio_2) {
    if (false == mrs_config->gpio_1->is_output()) {
      mrs_config->gpio_2->refresh(mrd_meridim);
    }
  }
}

bool control_meridian_board_lite_for_esp32(Meridim90Union &mrd_meridim) {
  // Control
  for (int i = 0; i < MERIDIAN_BOARD_LITE_FOR_ESP32_CONFIG_I2C_NUM; i++) {
    if (nullptr != mrs_config->i2c[i]) {
      mrs_config->i2c[i]->refresh(mrd_meridim);
    }
  }
  if (nullptr != mrs_config->servo_l) {
    mrs_config->servo_l->refresh(mrd_meridim);
  }
  if (nullptr != mrs_config->servo_r) {
    mrs_config->servo_r->refresh(mrd_meridim);
  }
}

bool output_meridian_board_lite_for_esp32(Meridim90Union &mrd_meridim) {
  // Output
  if (nullptr != mrs_config->gpio_1) {
    if (true == mrs_config->gpio_1->is_output()) {
      mrs_config->gpio_1->refresh(mrd_meridim);
    }
  }
  if (nullptr != mrs_config->gpio_2) {
    if (true == mrs_config->gpio_1->is_output()) {
      mrs_config->gpio_2->refresh(mrd_meridim);
    }
  }
  if (nullptr != mrs_config->wifi) {
    mrs_config->wifi->send(mrd_meridim);
  }
  if (nullptr != mrs_config->sd_card) {
    mrs_config->sd_card->refresh(mrd_meridim);
  }
  if (nullptr != mrs_config->eeprom) {
    mrs_config->eeprom->refresh(mrd_meridim);
  }

  delay(mrs_interval_ms);
}
