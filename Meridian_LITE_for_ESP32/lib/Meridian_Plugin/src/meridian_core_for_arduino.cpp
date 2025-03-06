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

#define DEBUG_MERIDIAN_CORE 0

namespace meridian {
namespace core {
namespace execution {
volatile bool count_frame_reset = false; // フレーム管理時計をリセットする

// ハードウェアタイマーとカウンタ用変数の定義
hw_timer_t *timer = NULL;                              // ハードウェアタイマーの設定
volatile SemaphoreHandle_t timer_semaphore;            // ハードウェアタイマー用のセマフォ
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED; // ハードウェアタイマー用のミューテックス
unsigned long count_frame = 0;                         // フレーム処理の完了時にカウントアップ
volatile unsigned long count_timer = 0;                // フレーム用タイマーのカウントアップ

/// @brief count_timerを保護しつつ1ずつインクリメント
void IRAM_ATTR frame_timer() {
  portENTER_CRITICAL_ISR(&timer_mux);
  count_timer++;
  portEXIT_CRITICAL_ISR(&timer_mux);
  xSemaphoreGiveFromISR(timer_semaphore, NULL); // セマフォを与える
}

void mrd_timer_setup(int a_duration_ms) {
  timer_semaphore = xSemaphoreCreateBinary(); // セマフォの作成
  timer = timerBegin(0, 80, true);            // タイマーの設定（1つ目のタイマーを使用, 分周比80）

  timerAttachInterrupt(timer, &frame_timer, true);    // frame_timer関数をタイマーの割り込みに登録
  timerAlarmWrite(timer, a_duration_ms * 1000, true); // タイマーを10msごとにトリガー
  timerAlarmEnable(timer);                            // タイマーを開始

  // タイマーの初期化
  count_frame = 0;
  portENTER_CRITICAL(&timer_mux);
  count_timer = 0;
  portEXIT_CRITICAL(&timer_mux);
}
void mrd_timer_delay() {
  count_frame++;
  while (true) {
    if (xSemaphoreTake(timer_semaphore, 0) == pdTRUE) {
      portENTER_CRITICAL(&timer_mux);
      unsigned long current_count_timer = count_timer; // ハードウェアタイマーの値を読む
      portEXIT_CRITICAL(&timer_mux);
      if (current_count_timer >= count_frame) {
        break;
      }
    }
  }

  // @[12-2] 必要に応じてフレームの遅延累積時間frameDelayをリセット
  if (count_frame_reset) {
    portENTER_CRITICAL(&timer_mux);
    count_frame = count_timer;
    portEXIT_CRITICAL(&timer_mux);
    count_frame_reset = false;
  }
}

void mrd_timer_reset() {
  count_frame_reset = true;
}

} // namespace execution
} // namespace core
} // namespace meridian
