/**
 * @file main.cpp
 * @brief main function
 * @version 1.2.0
 * @date 2025-01-21
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#include "board/fireflake_base_board_for_pico_w.hpp"
#include "pico/stdlib.h"

#include <stdio.h>

int main() {
  stdio_init_all();

  while (true) {
    printf("Hello, world!\n");
    sleep_ms(1000);
  }
}
