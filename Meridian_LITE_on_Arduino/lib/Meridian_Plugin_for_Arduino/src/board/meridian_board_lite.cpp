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

bool mrd_setup(mrd_entity *a_entity, mrd_parameters *a_param) {
  //////////////////////////////////////////////////////////
  // Initialize the shared memory
  //////////////////////////////////////////////////////////
  if (nullptr != a_param) {
    param = *a_param;
  }
  entity = a_entity;
  //////////////////////////////////////////////////////////
  // Start connecter
  //////////////////////////////////////////////////////////
  // Setting Serial
  if (PINS_DEFAULT_SERIAL0_RX != -1 && PINS_DEFAULT_SERIAL0_TX != -1) {
    if (PINS_DEFAULT_SERIAL0_RX == SOC_RX0 && PINS_DEFAULT_SERIAL0_TX == SOC_TX0) {
      Serial.begin(PINS_DEFAULT_SERIAL0_BAUD, PINS_DEFAULT_SERIAL0_CONFIG);
    } else {
      Serial.begin(PINS_DEFAULT_SERIAL0_BAUD, PINS_DEFAULT_SERIAL0_CONFIG, PINS_DEFAULT_SERIAL0_RX, PINS_DEFAULT_SERIAL0_TX);
    }
  }
#if SOC_UART_NUM > 1
  if (PINS_DEFAULT_SERIAL1_RX != -1 && PINS_DEFAULT_SERIAL1_TX != -1) {
    if (PINS_DEFAULT_SERIAL1_RX == RX1 && PINS_DEFAULT_SERIAL1_TX == TX1) {
      Serial1.begin(PINS_DEFAULT_SERIAL1_BAUD, PINS_DEFAULT_SERIAL1_CONFIG);
    } else {
      Serial1.begin(PINS_DEFAULT_SERIAL1_BAUD, PINS_DEFAULT_SERIAL1_CONFIG, PINS_DEFAULT_SERIAL1_RX, PINS_DEFAULT_SERIAL1_TX);
    }
  }
#endif
#if SOC_UART_NUM > 2
  if (PINS_DEFAULT_SERIAL2_RX != -1 && PINS_DEFAULT_SERIAL2_TX != -1) {
    if (PINS_DEFAULT_SERIAL2_RX == RX2 && PINS_DEFAULT_SERIAL2_TX == TX2) {
      Serial2.begin(PINS_DEFAULT_SERIAL2_BAUD, PINS_DEFAULT_SERIAL2_CONFIG);
    } else {
      Serial2.begin(PINS_DEFAULT_SERIAL2_BAUD, PINS_DEFAULT_SERIAL2_CONFIG, PINS_DEFAULT_SERIAL2_RX, PINS_DEFAULT_SERIAL2_TX);
    }
  }
#endif

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
      Wire.begin();
    } else {
      Wire.begin(PINS_DEFAULT_I2C_SDA, PINS_DEFAULT_I2C_SCL);
    }
    Wire.setClock(param.i2c_speed);
  }

  //////////////////////////////////////////////////////////
  // Setup
  //////////////////////////////////////////////////////////
  if (nullptr != entity) {
    for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
      if (nullptr != entity->plugin.gpio[i]) {
        entity->plugin.gpio[i]->setup();
      }
    }
  }
  return true;
}

Meridim90 mrd_input() {
  static Meridim90 a_meridim90;
  int result = 0;
  //////////////////////////////////////////////////////////
  // Copy the data from the shared memory
  //////////////////////////////////////////////////////////
  result = pthread_mutex_lock(&mutex);
  if (0 == result) {
    memcpy(&a_meridim90, &meridim90, sizeof(Meridim90));
  }
  result = pthread_mutex_unlock(&mutex);
  if (0 != result) {
    log_e(EXIT_FAILURE, result, "can not unlock");
  }

  //////////////////////////////////////////////////////////
  // Refresh
  //////////////////////////////////////////////////////////
  if (nullptr != entity) {
    for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
      if (nullptr != entity->plugin.gpio[i]) {
        if (true != entity->plugin.gpio[i]->is_output()) {
          entity->plugin.gpio[i]->refresh(a_meridim90);
        }
      }
    }
  }

  //////////////////////////////////////////////////////////
  return a_meridim90;
}
bool mrd_control(Meridim90 &mrd_meridim) {
  return true;
}
bool mrd_output(Meridim90 &mrd_meridim) {
  int result = 0;
  //////////////////////////////////////////////////////////
  // Copy the data to the shared memory
  //////////////////////////////////////////////////////////
  result = pthread_mutex_lock(&mutex);
  if (0 == result) {
    memcpy(&meridim90, &mrd_meridim, sizeof(Meridim90));
  }
  result = pthread_mutex_unlock(&mutex);
  if (0 != result) {
    log_e(EXIT_FAILURE, result, "can not unlock");
  }

  //////////////////////////////////////////////////////////
  // Refresh
  //////////////////////////////////////////////////////////
  if (nullptr != entity) {
    for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
      if (nullptr != entity->plugin.gpio[i]) {
        if (true == entity->plugin.gpio[i]->is_output()) {
          entity->plugin.gpio[i]->refresh(mrd_meridim);
        }
      }
    }
  }
  //////////////////////////////////////////////////////////
  return true;
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
