/**
 * @file meridian_core_for_arduino.cpp
 * @brief Arduino用Meridianのコアライブラリ
 * @version 1.2.0
 * @date 2025-03-06
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "meridian_core_for_arduino.hpp"
#include <Arduino.h>

#ifndef DEBUG_MERIDIAN_CORE
#define DEBUG_MERIDIAN_CORE 0
#endif

#define CALC_MS_TO_COUNTER(x) ((x) * (1000.0 / TIMER_SECTION_US))
#define CALC_COUNTER_TO_MS(x) ((x) * (TIMER_SECTION_US / 1000.0))

namespace meridian {
namespace core {
namespace execution {
volatile bool count_frame_reset = false; // フレーム管理時計をリセットする

// ハードウェアタイマーとカウンタ用変数の定義
const int TIMER_SECTION_US = 100;           ///! タイマーの分解能(100us)
const double TIMER_DELAY_RATE = 0.9;        ///! スリープモードの割合
hw_timer_t *timer = NULL;                   ///! ハードウェアタイマーの設定
uint8_t timer_no = 0;                       ///! タイマー番号
volatile SemaphoreHandle_t timer_semaphore; ///! ハードウェアタイマー用のセマフォ
volatile unsigned long count_timer = 0;     ///! フレーム用タイマーのカウントアップ
#if DEBUG_MERIDIAN_CORE
volatile unsigned long previous_count_timer = 0;
#endif

// portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED; ///! ハードウェアタイマー用のミューテックス

volatile bool flag_reset_count_timer = false;  ///! フレーム用タイマーのカウントリセット
volatile bool flag_reduce_count_timer = false; ///! フレーム用タイマーのカウントを減少させる

unsigned long timer_waypoint = CALC_MS_TO_COUNTER(10.0);                       ///! タイマーの目標値
unsigned long timer_delay_limit = CALC_MS_TO_COUNTER(10.0 * TIMER_DELAY_RATE); ///! スリープする閾値

int OVERFLOW_CLEAR_COUNT_MAX = 10; ///! オーバーフロー発生を確認してリセットするまでのカウント数

void mrd_timer_clear() {
  flag_reset_count_timer = true;
}
/// @brief count_timerを保護しつつ1ずつインクリメント
void IRAM_ATTR frame_timer() {
  count_timer++;
  if (flag_reduce_count_timer) {
    count_timer = count_timer % timer_waypoint;
    flag_reduce_count_timer = false;
#if DEBUG_MERIDIAN_CORE
    previous_count_timer = count_timer;
#endif
  }
  if (flag_reset_count_timer) {
    count_timer = 0;
    flag_reset_count_timer = false;
  }

  xSemaphoreGiveFromISR(timer_semaphore, NULL); // セマフォを与える
}

void mrd_timer_setup(int a_duration_ms) {
  timer_waypoint = (unsigned long)CALC_MS_TO_COUNTER(a_duration_ms);
  timer_delay_limit = (unsigned long)CALC_MS_TO_COUNTER(a_duration_ms * TIMER_DELAY_RATE);
  // タイマーの初期化
  mrd_timer_clear();
  // タイマーの設定
  timer_semaphore = xSemaphoreCreateBinary();                      // セマフォの作成
  timer = timerBegin(timer_no, getApbFrequency() / 1000000, true); // タイマーの設定(1つ目のタイマーを使用, 分周比80)
  timerAttachInterrupt(timer, &frame_timer, false);                // frame_timer関数をタイマーの割り込みに登録
  timerAlarmWrite(timer, TIMER_SECTION_US, true);                  // トリガー時間の設定
  timerAlarmEnable(timer);                                         // タイマーを開始
}
void mrd_timer_delay() {
  static int overflow_count = OVERFLOW_CLEAR_COUNT_MAX;
  unsigned long current_count_timer = 0;
#if DEBUG_MERIDIAN_CORE
  unsigned long running_count_timer = count_timer - previous_count_timer;
  int sleep_counting_hot = 0;
  int sleep_counting_delay = 0;
#endif
  int delay_time = 0;
  bool loop_count = false;
  bool flag_loop = true;
  while (flag_loop) {
    if (xSemaphoreTake(timer_semaphore, TIMER_SECTION_US) == pdTRUE) {
      // portENTER_CRITICAL(&timer_mux);
      current_count_timer = count_timer; // ハードウェアタイマーの値を読む
      // portEXIT_CRITICAL(&timer_mux);
      if (current_count_timer >= timer_waypoint) {
        flag_reduce_count_timer = true;
        flag_loop = false;
        break;
      }
    }
    if (true == flag_loop) {
      if (current_count_timer < timer_delay_limit) {
        // cold sleep
        delay_time = (int)CALC_COUNTER_TO_MS(timer_delay_limit - current_count_timer);
        delay_time += (0 != (timer_delay_limit % current_count_timer) ? 1 : 0);
        delay(delay_time);
#if DEBUG_MERIDIAN_CORE
        sleep_counting_delay += delay_time;
#endif
      } else {
        // hot sleep
#if DEBUG_MERIDIAN_CORE
        if (0 == sleep_counting_hot) {
          sleep_counting_hot = current_count_timer;
        }
#endif
      }
      loop_count = true;
    }
  }
  if (false == loop_count) {
    overflow_count--;
    if (overflow_count < 0) {
      mrd_timer_clear();
      overflow_count = OVERFLOW_CLEAR_COUNT_MAX;
    }
#if DEBUG_MERIDIAN_CORE
    Serial.printf("[E] cycle: [%5d][BOARDER:%d(R:%d)] %d(R:%3d H:%3d S:%3d)\n", millis(),
                  timer_waypoint, timer_delay_limit, current_count_timer,
                  running_count_timer,
                  (int)(current_count_timer - sleep_counting_hot),
                  (int)CALC_MS_TO_COUNTER(sleep_counting_delay));
  } else {
    Serial.printf("[i] cycle: [%5d][BOARDER:%d(R:%d)] %d(R:%3d H:%3d S:%3d)\n", millis(),
                  timer_waypoint, timer_delay_limit, current_count_timer,
                  running_count_timer,
                  (int)(current_count_timer - sleep_counting_hot),
                  (int)CALC_MS_TO_COUNTER(sleep_counting_delay));
#endif
  }
}

} // namespace execution
} // namespace core
} // namespace meridian

#undef CALC_MS_TO_COUNTER
#undef CALC_COUNTER_TO_MS
