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

namespace meridian {
namespace board {
namespace meridian_board_lite {
using namespace meridian::core::meridim;

Meridim90 meridim90;
mrd_entity *entity = nullptr;
mrd_parameters param;
mrd_parameters param_default = {
    .interval_ms = 10,
};

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

bool mrd_setup(mrd_entity *a_entity, mrd_parameters *a_param) {
  if (nullptr == a_param) {
    param = param_default;
  } else {
    param = *a_param;
  }
  entity = a_entity;
  //////////////////////////////////////////////////////////
  if (nullptr != entity) {
    for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
      if (nullptr != entity->gpio[i]) {
        entity->gpio[i]->setup();
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
  if (nullptr != entity) {
    for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
      if (nullptr != entity->gpio[i]) {
        if (true != entity->gpio[i]->is_output()) {
          entity->gpio[i]->refresh(a_meridim90);
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
  if (nullptr != entity) {
    for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
      if (nullptr != entity->gpio[i]) {
        if (true == entity->gpio[i]->is_output()) {
          entity->gpio[i]->refresh(mrd_meridim);
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
