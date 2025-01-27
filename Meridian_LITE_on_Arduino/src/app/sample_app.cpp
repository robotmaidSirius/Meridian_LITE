/**
 * @file sample_app.cpp
 * @brief 動作確認用のサンプルアプリケーションです
 * @version 1.2.0
 * @date 2025-01-27
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#include "sample_app.hpp"
#include <Arduino.h>

#include <mrd_module/gpio/mrd_module_gpio_out.hpp>
meridian::modules::plugin::MrdGpioOut gpio(2, 2, 0);

bool sample_app_setup(mrd_entity &entity) {
  log_i("");
  gpio.setup();
  return true;
}

bool sample_app_loop(Meridim90 &mrd_meridim, mrd_entity &entity) {
  static bool state = false;
  if (false == state) {
    gpio.write(1);
    entity.communication.diag->log_debug("----------------LED ON----------------");
    state = true;
  } else {
    gpio.write(0);
    entity.communication.diag->log_debug("----------------LED OFF----------------");
    state = false;
  }
  return true;
}
