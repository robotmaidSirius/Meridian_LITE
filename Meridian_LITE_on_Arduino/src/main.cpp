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
#include <mrd_module/gpio/mrd_analog_in.hpp>
#include <mrd_module/gpio/mrd_gpio_in.hpp>
#include <mrd_module/gpio/mrd_gpio_out.hpp>

mrd_parameters param = {
    .interval_ms = 10,
    //
};

mrd_entity entity = {
    .gpio = {
        new MrdAnalogIn(0, 0),
        new MrdAnalogIn(1, 1),
        new MrdGpioOut(2, 2),
        new MrdGpioOut(3, 3),
    },
};

void setup() {
  bool result = mrd_setup(&entity, &param);
}

void loop() {
  Meridim90 mrd_meridim = mrd_input();
  bool result = mrd_control(mrd_meridim);
  result &= mrd_output(mrd_meridim);
}
