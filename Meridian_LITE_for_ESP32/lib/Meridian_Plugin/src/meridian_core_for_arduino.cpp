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

namespace meridian {
namespace core {
namespace execution {
volatile bool count_frame_reset = false; // フレーム管理時計をリセットする

// ハードウェアタイマーとカウンタ用変数の定義
const int TIMER_SECTION_US = 100;           ///! タイマーの分解能(100us)
const double TIMER_HOT_SLEEP_RATE = 0.9;    ///! スリープモードの割合
hw_timer_t *timer = NULL;                   ///! ハードウェアタイマーの設定
uint8_t timer_no = 0;                       ///! タイマー番号
volatile SemaphoreHandle_t timer_semaphore; ///! ハードウェアタイマー用のセマフォ
volatile unsigned long count_timer = 0;     ///! フレーム用タイマーのカウントアップ

// portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED; ///! ハードウェアタイマー用のミューテックス

volatile bool flag_reset_count_timer = false;  ///! フレーム用タイマーのカウントリセット
volatile bool flag_reduce_count_timer = false; ///! フレーム用タイマーのカウントを減少させる

unsigned long timer_waypoint = 10.0 * (1000.0 / TIMER_SECTION_US);                               ///! タイマーの目標値
unsigned long timer_waypoint_keep = (10.0 * TIMER_HOT_SLEEP_RATE) * (1000.0 / TIMER_SECTION_US); ///! スリープモードの目標値

int OVERFLOW_CLEAR_COUNT_MAX = 10;

void mrd_timer_clear() {
  flag_reset_count_timer = true;
}
/// @brief count_timerを保護しつつ1ずつインクリメント
void IRAM_ATTR frame_timer() {
  count_timer++;
  if (flag_reduce_count_timer) {
    count_timer = count_timer % timer_waypoint;
    flag_reduce_count_timer = false;
  }
  if (flag_reset_count_timer) {
    count_timer = 0;
    flag_reset_count_timer = false;
  }

  xSemaphoreGiveFromISR(timer_semaphore, NULL); // セマフォを与える
}

void mrd_timer_setup(int a_duration_ms) {
  timer_waypoint = (unsigned long)(a_duration_ms * (1000.0 / TIMER_SECTION_US));
  timer_waypoint_keep = (unsigned long)((a_duration_ms * TIMER_HOT_SLEEP_RATE) * (1000.0 / TIMER_SECTION_US));
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
  int sleep_counting_hot = 0;
  int sleep_counting_delay = 0;
#endif
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
      if (current_count_timer < timer_waypoint_keep) {
        // cold sleep
        delay(1);
#if DEBUG_MERIDIAN_CORE
        sleep_counting_delay++;
      } else {
        // hot sleep
        sleep_counting_hot++;
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
    Serial.printf("[E] cycle: [%5d][BOARDER:%d(R:%d)] %d(skip:%3d +%3d ms)\n", millis(), timer_waypoint, timer_waypoint_keep, current_count_timer, sleep_counting_hot, sleep_counting_delay);
  } else {
    Serial.printf("[i] cycle: [%5d][BOARDER:%d(R:%d)] %d(skip:%3d +%3d ms)\n", millis(), timer_waypoint, timer_waypoint_keep, current_count_timer, sleep_counting_hot, sleep_counting_delay);
#endif
  }
}

} // namespace execution
} // namespace core
} // namespace meridian
