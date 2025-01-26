/**
 * @file meridian_board_lite.cpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#include "board/meridian_board_lite.hpp"
#include <Arduino.h>
#include <Wire.h>

namespace meridian {
namespace board {
namespace meridian_board_lite {
using namespace meridian::core::meridim;

Meridim90 meridim90;
mrd_entity *entity = nullptr;
mrd_parameters param;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

bool board_setup(mrd_entity *a_entity, mrd_parameters *a_param) {
  bool result = true;
  //////////////////////////////////////////////////////////
  // Initialize the shared memory
  //////////////////////////////////////////////////////////
  if (nullptr != a_param) {
    param = *a_param;
  }
  entity = a_entity;
  //////////////////////////////////////////////////////////
  // Arduino Setup
  //////////////////////////////////////////////////////////
  // Setting Serial
  if (PINS_DEFAULT_SERIAL0_RX != -1 && PINS_DEFAULT_SERIAL0_TX != -1) {
    if (PINS_DEFAULT_SERIAL0_RX == SOC_RX0 && PINS_DEFAULT_SERIAL0_TX == SOC_TX0) {
      Serial.begin(BOARD_SETTING_DEFAULT_SERIAL0_BAUD, BOARD_SETTING_DEFAULT_SERIAL0_CONFIG);
    } else {
      Serial.begin(BOARD_SETTING_DEFAULT_SERIAL0_BAUD, BOARD_SETTING_DEFAULT_SERIAL0_CONFIG, PINS_DEFAULT_SERIAL0_RX, PINS_DEFAULT_SERIAL0_TX);
    }
  }
#if SOC_UART_NUM > 1
  if (PINS_DEFAULT_SERIAL1_RX != -1 && PINS_DEFAULT_SERIAL1_TX != -1) {
    if (PINS_DEFAULT_SERIAL1_RX == RX1 && PINS_DEFAULT_SERIAL1_TX == TX1) {
      Serial1.begin(BOARD_SETTING_DEFAULT_SERIAL1_BAUD, BOARD_SETTING_DEFAULT_SERIAL1_CONFIG);
    } else {
      Serial1.begin(BOARD_SETTING_DEFAULT_SERIAL1_BAUD, BOARD_SETTING_DEFAULT_SERIAL1_CONFIG, PINS_DEFAULT_SERIAL1_RX, PINS_DEFAULT_SERIAL1_TX);
    }
  }
#endif
#if SOC_UART_NUM > 2
  if (PINS_DEFAULT_SERIAL2_RX != -1 && PINS_DEFAULT_SERIAL2_TX != -1) {
    if (PINS_DEFAULT_SERIAL2_RX == RX2 && PINS_DEFAULT_SERIAL2_TX == TX2) {
      Serial2.begin(BOARD_SETTING_DEFAULT_SERIAL2_BAUD, BOARD_SETTING_DEFAULT_SERIAL2_CONFIG);
    } else {
      Serial2.begin(BOARD_SETTING_DEFAULT_SERIAL2_BAUD, BOARD_SETTING_DEFAULT_SERIAL2_CONFIG, PINS_DEFAULT_SERIAL2_RX, PINS_DEFAULT_SERIAL2_TX);
    }
  }
#endif
  if (nullptr != entity) {
    if (nullptr != entity->communication.diag) {
      result = entity->communication.diag->setup();
    } else {
      result = false;
    }
    if (false == result) {
      entity->communication.diag = new IMeridianDiagnostic();
      entity->communication.diag->setup();
      result = true;
    }
    entity->communication.diag->printf(
        result ? IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_INFO : IMeridianDiagnostic::OUTPUT_LOG_LEVEL::LEVEL_ERROR,
        "[%s] Setup/communication::diagnostic", result ? "Succeeded" : "Failed");
  }

  // Setting I2C
  bool flag_ic2_begin = false;
  if (nullptr != entity) {
    for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
      if (nullptr != entity->plugin.gpio[i]) {
        flag_ic2_begin = true;
        break;
      }
    }
  }
  if (true == flag_ic2_begin) {
    if (PINS_DEFAULT_I2C_SDA == SDA && PINS_DEFAULT_I2C_SCL == SCL) {
      result &= Wire.begin();
    } else {
      result &= Wire.begin(PINS_DEFAULT_I2C_SDA, PINS_DEFAULT_I2C_SCL);
    }
    result &= Wire.setClock(param.i2c_speed);
  }
  // Setting SPI
  // SPI.begin();

  //////////////////////////////////////////////////////////
  // Setup Communication
  //////////////////////////////////////////////////////////
  if (true == result) {
    if (nullptr != entity) {
      if (nullptr != entity->communication.con) {
        entity->communication.con->set_diagnostic(*entity->communication.diag);
        result &= entity->communication.con->setup();
      }
      entity->communication.diag->log_debug("[%s] Setup/communication::conversation", result ? "Succeeded" : "Failed");
    }

    //////////////////////////////////////////////////////////
    // Setup Plugin
    //////////////////////////////////////////////////////////
    if (nullptr != entity) {
      for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
        if (nullptr != entity->plugin.gpio[i]) {
          entity->plugin.gpio[i]->set_diagnostic(*entity->communication.diag);
          result &= entity->plugin.gpio[i]->setup();
          entity->communication.diag->log_debug("[%s] Setup/plugin::gpio[%d]", result ? "Succeeded" : "Failed", i);
        }
      }
      for (int i = 0; i < MERIDIAN_BOARD_LITE_ANALOG_NUM; i++) {
        if (nullptr != entity->plugin.analog[i]) {
          entity->plugin.analog[i]->set_diagnostic(*entity->communication.diag);
          result &= entity->plugin.analog[i]->setup();
          entity->communication.diag->log_debug("[%s] Setup/plugin::analog[%d]", result ? "Succeeded" : "Failed", i);
        }
      }
      for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
        if (nullptr != entity->plugin.i2c[i]) {
          entity->plugin.i2c[i]->set_diagnostic(*entity->communication.diag);
          result &= entity->plugin.i2c[i]->setup();
          entity->communication.diag->log_debug("[%s] Setup/plugin::i2c[%d]", result ? "Succeeded" : "Failed", i);
        }
      }
      if (nullptr != entity->plugin.spi_sd_card) {
        entity->plugin.spi_sd_card->set_diagnostic(*entity->communication.diag);
        result &= entity->plugin.spi_sd_card->setup();
        entity->communication.diag->log_debug("[%s] Setup/plugin::spi_sd_card", result ? "Succeeded" : "Failed");
      }
      if (nullptr != entity->plugin.spi) {
        entity->plugin.spi->set_diagnostic(*entity->communication.diag);
        result &= entity->plugin.spi->setup();
        entity->communication.diag->log_debug("[%s] Setup/plugin::spi", result ? "Succeeded" : "Failed");
      }
      if (nullptr != entity->plugin.servo_left) {
        entity->plugin.servo_left->set_diagnostic(*entity->communication.diag);
        result &= entity->plugin.servo_left->setup();
        entity->communication.diag->log_debug("[%s] Setup/plugin::servo_left", result ? "Succeeded" : "Failed");
      }
      if (nullptr != entity->plugin.servo_right) {
        entity->plugin.servo_right->set_diagnostic(*entity->communication.diag);
        result &= entity->plugin.servo_right->setup();
        entity->communication.diag->log_debug("[%s] Setup/plugin::servo_right", result ? "Succeeded" : "Failed");
      }
    }
  }
  entity->communication.diag->log_debug("[%s] Setup", result ? "Succeeded" : "Failed");
  return result;
}

Meridim90 mrd_input() {
  static Meridim90 a_meridim90;
  //////////////////////////////////////////////////////////
  // Copy the data from the shared memory
  //////////////////////////////////////////////////////////
  int result_lock = pthread_mutex_lock(&mutex);
  if (0 == result_lock) {
    memcpy(&a_meridim90, &meridim90, sizeof(Meridim90));
  }
  result_lock = pthread_mutex_unlock(&mutex);
  if (0 != result_lock) {
    log_e("[%d] %s", result_lock, "can not unlock");
  }
  bool result = true;
  if (nullptr != entity) {
    //////////////////////////////////////////////////////////
    // Input Communication
    //////////////////////////////////////////////////////////
    if (nullptr != entity->communication.con) {
      result &= entity->communication.con->received(a_meridim90);
    }
    //////////////////////////////////////////////////////////
    // Input Plugin
    //////////////////////////////////////////////////////////
    for (int i = 0; i < MERIDIAN_BOARD_LITE_ANALOG_NUM; i++) {
      if (nullptr != entity->plugin.analog[i]) {
        result &= entity->plugin.analog[i]->input(a_meridim90);
      }
    }
    for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
      if (nullptr != entity->plugin.gpio[i]) {
        result &= entity->plugin.gpio[i]->input(a_meridim90);
      }
    }
    for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
      if (nullptr != entity->plugin.i2c[i]) {
        result &= entity->plugin.i2c[i]->input(a_meridim90);
      }
    }
    if (nullptr != entity->plugin.servo_left) {
      result &= entity->plugin.servo_left->input(a_meridim90);
    }
    if (nullptr != entity->plugin.servo_right) {
      result &= entity->plugin.servo_right->input(a_meridim90);
    }
    if (nullptr != entity->plugin.spi) {
      result &= entity->plugin.spi->input(a_meridim90);
    }
    if (nullptr != entity->plugin.spi_sd_card) {
      result &= entity->plugin.spi_sd_card->input(a_meridim90);
    }
  }
  return a_meridim90;
}
bool mrd_processing(Meridim90 &a_meridim90) {
  bool result = true;
  //////////////////////////////////////////////////////////
  // Processing Plugin
  //////////////////////////////////////////////////////////
  for (int i = 0; i < MERIDIAN_BOARD_LITE_ANALOG_NUM; i++) {
    if (nullptr != entity->plugin.analog[i]) {
      result &= entity->plugin.analog[i]->processing(a_meridim90);
    }
  }
  for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
    if (nullptr != entity->plugin.gpio[i]) {
      result &= entity->plugin.gpio[i]->processing(a_meridim90);
    }
  }
  for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
    if (nullptr != entity->plugin.i2c[i]) {
      result &= entity->plugin.i2c[i]->processing(a_meridim90);
    }
  }
  if (nullptr != entity->plugin.servo_left) {
    result &= entity->plugin.servo_left->processing(a_meridim90);
  }
  if (nullptr != entity->plugin.servo_right) {
    result &= entity->plugin.servo_right->processing(a_meridim90);
  }
  if (nullptr != entity->plugin.spi) {
    result &= entity->plugin.spi->processing(a_meridim90);
  }
  if (nullptr != entity->plugin.spi_sd_card) {
    result &= entity->plugin.spi_sd_card->processing(a_meridim90);
  }
  return result;
}
bool mrd_output(Meridim90 &a_meridim90) {
  bool result = true;
  //////////////////////////////////////////////////////////
  // Output Plugin
  //////////////////////////////////////////////////////////
  for (int i = 0; i < MERIDIAN_BOARD_LITE_ANALOG_NUM; i++) {
    if (nullptr != entity->plugin.analog[i]) {
      result &= entity->plugin.analog[i]->output(a_meridim90);
    }
  }
  for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
    if (nullptr != entity->plugin.gpio[i]) {
      result &= entity->plugin.gpio[i]->output(a_meridim90);
    }
  }
  for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
    if (nullptr != entity->plugin.i2c[i]) {
      result &= entity->plugin.i2c[i]->output(a_meridim90);
    }
  }
  if (nullptr != entity->plugin.servo_left) {
    result &= entity->plugin.servo_left->output(a_meridim90);
  }
  if (nullptr != entity->plugin.servo_right) {
    result &= entity->plugin.servo_right->output(a_meridim90);
  }
  if (nullptr != entity->plugin.spi) {
    result &= entity->plugin.spi->output(a_meridim90);
  }
  if (nullptr != entity->plugin.spi_sd_card) {
    result &= entity->plugin.spi_sd_card->output(a_meridim90);
  }
  if (true == result) {
    //////////////////////////////////////////////////////////
    // Output Communication
    //////////////////////////////////////////////////////////
    if (nullptr != entity->communication.con) {
      result &= entity->communication.con->send(a_meridim90);
    }
    //////////////////////////////////////////////////////////
    // Copy the data to the shared memory
    //////////////////////////////////////////////////////////
    int result_lock = pthread_mutex_lock(&mutex);
    if (0 == result_lock) {
      memcpy(&meridim90, &a_meridim90, sizeof(Meridim90));
    }
    result_lock = pthread_mutex_unlock(&mutex);
    if (0 != result_lock) {
      log_e("[%d] %s", result_lock, "can not unlock");
    }
  }
  return result;
}

} // namespace meridian_board_lite
} // namespace board
} // namespace meridian

#if 0

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

const size_t loop_max = 65535;

pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;
int counter = 0;

void f1() {
  size_t i;

  for (i = 0; i < loop_max; i++) {
#ifndef NOLOCK
    int r;
    r = pthread_mutex_lock(&m);
    if (r != 0) {
      log_e("%d/%s", r, "can not lock");
    }
#endif
    counter++;
#ifndef NOLOCK
    r = pthread_mutex_unlock(&m);
    if (r != 0) {
      log_e("%d/%s", r, "can not unlock");
    }
#endif
  }
}

void f2() {
  size_t i;

  for (i = 0; i < loop_max; i++) {
#ifndef NOLOCK
    if (pthread_mutex_lock(&m) != 0) {
      log_e("can not lock");
    }
#endif
    counter++;
#ifndef NOLOCK
    if (pthread_mutex_unlock(&m) != 0) {
      log_e("can not unlock");
    }
#endif
  }
}

int mutex_main(int argc, char *argv[]) {
  pthread_t thread1, thread2;
  int ret1 = pthread_create(&thread1, NULL, (void *)f1, NULL);
  int ret2 = pthread_create(&thread2, NULL, (void *)f2, NULL);
  if (ret1 != 0) {
    log_e("can not create thread 1: %s", strerror(ret1));
  }
  if (ret2 != 0) {
    log_e("can not create thread 2: %s", strerror(ret2));
  }

  printf("execute pthread_join thread1\n");
  ret1 = pthread_join(thread1, NULL);
  if (ret1 != 0) {
    log_e("%d/%s", ret1, "can not join thread 1");
  }

  printf("execute pthread_join thread2\n");
  ret2 = pthread_join(thread2, NULL);
  if (ret2 != 0) {
    log_e("%d/%s", ret2, "can not join thread 2");
  }

  printf("done\n");
  printf("%d\n", counter);

  pthread_mutex_destroy(&m);
  return 0;
}

#endif
