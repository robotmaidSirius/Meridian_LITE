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
#define DEBUG_BOARD_LED 1

#if DEBUG_BOARD_LED
meridian::modules::plugin::MrdGpioOut gpio(2, 2, 0);
#endif

bool sample_app_setup(mrd_entity &entity) {
  log_i("");
#if DEBUG_BOARD_LED
  gpio.setup();
#endif
  entity.plugin.gpio[1]->write(1);
  return true;
}

bool sample_app_loop(Meridim90 &mrd_meridim, mrd_entity &entity) {
  // LED Test
  static bool led_state = false;
  if (false == led_state) {
#if DEBUG_BOARD_LED
    gpio.write(1);
    gpio.output(mrd_meridim);
#else
    entity.plugin.gpio[0]->write(1);
#endif
    entity.communication.diag->log_debug("---------------- LED ON ----------------");
    led_state = true;
  } else {
#if DEBUG_BOARD_LED
    gpio.write(0);
    gpio.output(mrd_meridim);
#else
    entity.plugin.gpio[0]->write(0);
#endif
    entity.communication.diag->log_debug("---------------- LED OFF----------------");
    led_state = false;
  }
  // Log Test
  static int log_shift = -1;
  static bool flag_log = true;
  log_shift++;
  IMeridianDiagnostic::OUTPUT_LOG_LEVEL log_level;
  int buf_log_shift = log_shift;
  switch (log_shift) {
  case 0:
    log_level = IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_ALL;
    break;
  case 1:
    log_level = IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_TRACE;
    break;
  case 2:
    log_level = IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_DEBUG;
    break;
  case 3:
    log_level = IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_INFO;
    break;
  case 4:
    log_level = IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_WARN;
    break;
  case 5:
    log_level = IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_ERROR;
    break;
  case 6:
    log_level = IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_FATAL;
    break;
  case 7:
    log_level = IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_OPERATIONAL;
    break;
  case 8:
  default:
    log_level = IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_OFF;
    log_shift = -1;
    if (true == flag_log) {
      flag_log = false;
      entity.communication.diag->disable();
    } else {
      flag_log = true;
      entity.communication.diag->enable();
    }
    break;
  }
  log_i("---------------- Log[%d/%s:%s] ----------------", buf_log_shift, entity.communication.diag->get_text_level(log_level), flag_log ? "Enable" : "Disable");
  entity.communication.diag->set_level(log_level);

  entity.communication.diag->log_trace("test[%s,%d]", __FILE__, __LINE__);
  entity.communication.diag->log_debug("test[%s,%d]", __FILE__, __LINE__);
  entity.communication.diag->log_info("test[%s,%d]", __FILE__, __LINE__);
  entity.communication.diag->log_warn("test[%s,%d]", __FILE__, __LINE__);
  entity.communication.diag->log_error("test[%s,%d]", __FILE__, __LINE__);
  entity.communication.diag->log_fatal("test[%s,%d]", __FILE__, __LINE__);
  entity.communication.diag->log("<MES> test[%s,%d]\n", __FILE__, __LINE__);

  return true;
}
