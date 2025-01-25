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
        .diag = new MrdDiagnosticUart(Serial, 115200),
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
        .spi_outside = nullptr,
        .spi_inside = nullptr,
        .servo_left = nullptr,
        .servo_right = nullptr,
    },
};

void setup() {
  mrd_parameters param;
  param.interval_ms = 10;
  param.i2c_speed = 400000UL;

  bool result = mrd_setup(&entity, &param);
}

void loop() {
  Meridim90 mrd_meridim = mrd_input();
  bool result = mrd_control(mrd_meridim);
  result &= mrd_output(mrd_meridim);
}
