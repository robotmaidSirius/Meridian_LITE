/**
 * @file main.cpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#include <board/meridian_board_lite.hpp>
#include <mrd_communication/mrd_conversation_wifi.hpp>
#include <mrd_communication/mrd_diagnostic_uart.hpp>
#include <mrd_module/ahrs/mrd_module_ahrs_BNO055.hpp>
#include <mrd_module/gpio/mrd_module_analog_in.hpp>
#include <mrd_module/gpio/mrd_module_gpio_in.hpp>
#include <mrd_module/gpio/mrd_module_gpio_out.hpp>

mrd_entity entity = {
    .communication = {
        .con = new MrdConversationWifi(),
        .diag = new MrdDiagnosticUart(&Serial,
                                      BOARD_SETTING_DEFAULT_SERIAL0_BAUD,
                                      MrdDiagnosticUart::OUTPUT_LOG_LEVEL::LEVEL_DEBUG),
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
            nullptr,
            nullptr,
            nullptr,
            nullptr,
        },
        .spi_sd_card = nullptr,
        .spi = nullptr,
        .servo_left = nullptr,
        .servo_right = nullptr,
    },
};

void setup() {
  if (false == board_setup(&entity, new mrd_parameters())) {
    while (true) {
      log_e("Board setup failed.");
      delay(1000);
    }
  }
}

void loop() {
  // データの取得
  Meridim90 mrd_meridim = mrd_input();

  // 変換処理
  bool result = mrd_processing(mrd_meridim);

  // 出力処理
  result &= mrd_output(mrd_meridim);
  delay(1000);
}
