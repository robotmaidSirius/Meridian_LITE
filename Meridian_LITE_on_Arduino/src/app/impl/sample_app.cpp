/**
 * @file sample_app.cpp
 * @brief 動作確認用のサンプルアプリケーションです
 * @version 1.2.0
 * @date 2025-01-27
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#include "../sample_app.hpp"
#include <Arduino.h>

#include <mrd_module/gpio/mrd_module_gpio_out.hpp>
////////////////////////////////////////////////////////////////////////////////////////////////
// テスト関数
////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief LED点灯テスト (再定義)
void test_led_board(Meridim90 &mrd_meridim, mrd_entity &entity) {
  // [setup]
  static bool init = true;
  static meridian::modules::plugin::MrdGpioOut gpio(2, 2, 0);
  if (init) {
    gpio.setup();
    init = false;
  }
  // [loop]
  static bool led_state = false;
  if (false == led_state) {
    gpio.write(1);
    gpio.output(mrd_meridim);
    entity.communication.diag->log_debug("---------------- LED ON ----------------");
    led_state = true;
  } else {
    gpio.write(0);
    gpio.output(mrd_meridim);
    entity.communication.diag->log_debug("---------------- LED OFF----------------");
    led_state = false;
  }
}

/// @brief LED点灯テスト (entityを使用)
void test_led_entity(Meridim90 &mrd_meridim, mrd_entity &entity) {
  // [setup]
  static bool init = true;
  if (init) {
    init = false;
  }
  // [loop]
  static bool led_state = false;
  if (false == led_state) {
    static_cast<meridian::modules::plugin::MrdGpioOut *>(entity.plugin.gpio[0])->write(true);
    entity.communication.diag->log_debug("---------------- LED ON ----------------");
    led_state = true;
  } else {
    static_cast<meridian::modules::plugin::MrdGpioOut *>(entity.plugin.gpio[0])->write(false);
    entity.communication.diag->log_debug("---------------- LED OFF----------------");
    led_state = false;
  }
}

/// @brief ログ出力テスト
void test_log(Meridim90 &mrd_meridim, mrd_entity &entity) {
  // [setup]
  static bool init = true;
  if (init) {
    init = false;
  }
  // [loop]
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
    if (flag_log) {
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
}

////////////////////////////////////////////////////////////////////////////////////////////////
// 外部関数
////////////////////////////////////////////////////////////////////////////////////////////////
bool sample_app_setup(mrd_entity &entity) {
  static_cast<meridian::modules::plugin::MrdGpioOut *>(entity.plugin.gpio[1])->write(true);
  return true;
}
#define TEST_ENABLE_LED        0
#define TEST_ENABLE_OUTPUT_SEQ 0
#define TEST_ENABLE_BOARD_LED  0
#define TEST_ENABLE_LOG        0
bool sample_app_loop(Meridim90 &mrd_meridim, mrd_entity &entity) {

  // LED Test
#if TEST_ENABLE_LED
#if TEST_ENABLE_BOARD_LED
  test_led_board(mrd_meridim, entity);
#else
  test_led_entity(mrd_meridim, entity);
#endif
#endif

#if TEST_ENABLE_LOG
  // Log Test
  test_log(mrd_meridim, entity);
#endif

#if TEST_ENABLE_OUTPUT_SEQ
  log_i("SEQ[%04X] CH-SUM[%04X]", mrd_meridim.sequential, mrd_meridim.checksum);
#endif
  return true;
}
