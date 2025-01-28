/**
 * @file main.cpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#include "app/sample_app.hpp"
#include "app/sample_app_default.hpp"
#include "keys.h"
#include <board/meridian_board_lite.hpp>
//////////////////////////////////////////////////////////////////////////
// mrd_communication
//////////////////////////////////////////////////////////////////////////
#include <mrd_communication/mrd_conversation_wifi.hpp>
#include <mrd_communication/mrd_diagnostic_uart.hpp>

//////////////////////////////////////////////////////////////////////////
// mrd_module
//////////////////////////////////////////////////////////////////////////
#if defined(MODULE_AHRS_BNO055)
#include <mrd_module/ahrs/mrd_module_ahrs_BNO055.hpp>
#elif defined(MODULE_AHRS_MPU6050)
#include <mrd_module/ahrs/mrd_module_imu_MPU6050.hpp>
#endif

#if defined(MODULE_FS_EEPROM)
#include <mrd_module/filesystem/mrd_module_eeprom.hpp>
#endif
#if defined(MODULE_FS_SD_CARD)
#include <mrd_module/filesystem/mrd_module_sd.hpp>
#endif

#if defined(MODULE_PAD_WIIMOTE)
#include <mrd_module/pad/mrd_module_pad_wiimote.hpp>
#elif defined(MODULE_PAD_KRC5FH)
#include <mrd_module/pad/mrd_module_pad_krc5fh.hpp>
#endif

#include <mrd_module/gpio/mrd_module_analog_in.hpp>
#include <mrd_module/gpio/mrd_module_gpio_in.hpp>
#include <mrd_module/gpio/mrd_module_gpio_out.hpp>

//////////////////////////////////////////////////////////////////////////
#define PINS_BOARD_LED_CONNECT (2)
#define PINS_BOARD_LED_SIGNAL  (0xFF)
#define SETTING_LOG_LEVEL      (MrdDiagnosticUart::OUTPUT_LOG_LEVEL::LEVEL_DEBUG)
//////////////////////////////////////////////////////////////////////////
// 使用するモジュールの設定
//////////////////////////////////////////////////////////////////////////
sample_app_default app_default;
MrdConversationWifi con_wifi(
#if 0xFF != PINS_BOARD_LED_CONNECT
    new MrdGpioOut(PINS_BOARD_LED_CONNECT),
#else
    nullptr,
#endif
#if 0xFF != PINS_BOARD_LED_SIGNAL
    new MrdGpioOut(PINS_BOARD_LED_SIGNAL)
#else
    nullptr
#endif
);
MrdDiagnosticUart diag_uart(&Serial, BOARD_SETTING_DEFAULT_SERIAL0_BAUD, SETTING_LOG_LEVEL);
mrd_entity entity = {
    .communication = {
        .con = &con_wifi,
        .diag = &diag_uart,
    },
    .plugin = {
        .analog = {
            new MrdAnalogIn(PINS_DEFAULT_ANALOG_IN_1, 0),
            new MrdAnalogIn(PINS_DEFAULT_ANALOG_IN_2, 1),
        },
        .gpio = {
            new MrdGpioOut(PINS_DEFAULT_GPIO_1, 2, 0),
            new MrdGpioOut(PINS_DEFAULT_GPIO_2, 2, 1),
        },
        .i2c = {
#if defined(MODULE_AHRS_BNO055)
            new MrdAhrsBNO055(0x00),
#elif defined(MODULE_AHRS_MPU6050)
            new MrdAhrsMPU6050(0x00),
#else
            nullptr,
#endif
            nullptr,
            nullptr,
            nullptr,
        },
        .eeprom =
#if defined(MODULE_FS_SD_CARD)
            new MrdEEPROM(),
#else
            nullptr,
#endif
        .sd_card =
#if defined(MODULE_FS_SD_CARD)
            new MrdSdCard(),
#else
            nullptr,
#endif
        .spi = nullptr,
        .servo_left = nullptr,
        .servo_right = nullptr,
        .pad = nullptr,
    },
};

void setup_error() {
  while (true) {
    log_e("Board setup failed.");
    if (false == diagnosis.communication.con.initalized) {
      log_e("  communication::conversation");
    }
    if (false == diagnosis.communication.diag.initalized) {
      log_e("  communication::diagnostic");
    }
    for (int i = 0; i < MERIDIAN_BOARD_LITE_ANALOG_NUM; i++) {
      if (false == diagnosis.plugin.analog[i].initalized) {
        log_e("  plugin::analog[%d]", i);
      }
    }
    for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
      if (false == diagnosis.plugin.gpio[i].initalized) {
        log_e("  plugin::gpio[%d]", i);
      }
    }
    for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
      if (false == diagnosis.plugin.i2c[i].initalized) {
        log_e("  plugin::i2c[%d]", i);
      }
    }
    if (false == diagnosis.plugin.eeprom.initalized) {
      log_e("  plugin::eeprom");
    }
    if (false == diagnosis.plugin.sd_card.initalized) {
      log_e("  plugin::sd_card");
    }
    if (false == diagnosis.plugin.spi.initalized) {
      log_e("  plugin::common_spi");
    }
    if ((false == diagnosis.plugin.servo_left.initalized) || (false == diagnosis.plugin.servo_right.initalized)) {
      log_e("  plugin::servo %s%s",
            (false == diagnosis.plugin.servo_left.initalized) ? "L" : "",
            (false == diagnosis.plugin.servo_right.initalized) ? "R" : "");
    }
    if (false == diagnosis.plugin.pad.initalized) {
      log_e("  plugin::pad");
    }

    delay(1000);
  }
}

/// @brief セットアップ関数
void setup() {
  bool result = false;
  //////////////////////
  app_default.setup();
  entity.plugin.pad = &app_default.pad;
  entity.plugin.servo_left = &app_default.servo_left;
  entity.plugin.servo_right = &app_default.servo_right;

  //////////////////////
  if (true == board_setup(&entity, new mrd_parameters())) {
    result = sample_app_setup(entity);
  }
  if (true == result) {
    con_wifi.add_target(WIFI_SEND_IP, UDP_SEND_PORT);
    result = con_wifi.connect(WIFI_AP_SSID, WIFI_AP_PASS, UDP_RESV_PORT);
  }

  if (false == result) {
    setup_error();
  } else {
    diag_uart.log_info("This machine IP: %s\n", con_wifi.get_ip_address());
    for (int i = 0; i < MrdConversationWifi::NUMBER_ALLOWED; i++) {
      if (0 != con_wifi.target[i].port) {
        diag_uart.log_info("  Send Target[%s:%d]\n", con_wifi.target[i].ip.toString().c_str(), con_wifi.target[i].port);
      }
    }
  }
}

/// @brief メインループ
void loop() {
  // データの取得
  Meridim90 mrd_meridim = mrd_input();

  // アプリ処理
  bool result = sample_app_loop(mrd_meridim, entity);
  if (false == result) {
    log_e("======== application failed.");
  }
  //////////////////////
  app_default.loop(mrd_meridim);

  // 出力処理
  result &= mrd_output(mrd_meridim);
  if (false == result) {
    log_e("======== output failed.");
  }
  // 待機
  delay(mrd_delay());
}
