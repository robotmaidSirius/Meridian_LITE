/**
 * @file meridian_core_for_arduino.cpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-29
 *
 * @copyright Copyright (c) 2025-.
 *
 */

#include "meridian_core.hpp"
#include <Arduino.h>

namespace meridian {
namespace core {
namespace execution {

//------------------------------------------------------------------------------------
//  タイムアウト監視用タイマー
//------------------------------------------------------------------------------------
// タイマーの開始状態を保持するための静的変数
static unsigned long timeout_start = 0;
static bool flg_timer_started = false; // タイマーが開始されたかどうかのフラグ

/// @brief 指定されたミリ秒のタイムアウトを監視する. mrd_timeout_resetとセットで使う.
/// @param a_limit タイムアウトまでの時間(ms)
/// @return タイムアウトでtrueを返す.
bool mrd_timeout_check(unsigned long a_limit) {
  // タイマーが開始されていない場合は現在の時間を記録してタイマーを開始
  if (!flg_timer_started) {
    timeout_start = millis();
    flg_timer_started = true; // タイムアウト監視開始フラグをアゲる
  }

  unsigned long current_time = millis(); // 現在の時間を取得

  if (current_time - timeout_start >= a_limit) { // 指定された時間が経過しているかチェック
    flg_timer_started = false;                   // タイムアウト監視開始フラグをサゲる
    return true;                                 // 指定された時間が経過していれば true を返す
  }

  return false; // まだ時間が経過していなければ false を返す
}

/// @brief タイムアウト監視開始フラグをリセットする. mrd_timeout_checkとセットで使う.
void mrd_timeout_reset() {
  flg_timer_started = false; // タイマーをリセットして次回の呼び出しに備える
}

//------------------------------------------------------------------------------------
//  タイムアウト監視用タイマー
//------------------------------------------------------------------------------------

// ハードウェアタイマーとカウンタ用変数の定義
const int TIMER_SECTION_US = 100;                              ///! タイマーの分解能(100us)
hw_timer_t *timer = NULL;                                      ///! ハードウェアタイマーの設定
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED;         ///! ハードウェアタイマー用のミューテックス
volatile SemaphoreHandle_t timer_semaphore;                    ///! ハードウェアタイマー用のセマフォ
volatile unsigned long count_timer = 0;                        ///! フレーム用タイマーのカウントアップ
unsigned long timer_waypoint = 10 * (1000 / TIMER_SECTION_US); ///! タイマーの目標値

void mrd_timer_clear() {
  portENTER_CRITICAL(&timer_mux);
  count_timer = 0;
  portEXIT_CRITICAL(&timer_mux);
}
/// @brief count_timerを保護しつつ1ずつインクリメント
void IRAM_ATTR frame_timer() {
  portENTER_CRITICAL_ISR(&timer_mux);
  count_timer++;
  portEXIT_CRITICAL_ISR(&timer_mux);
  xSemaphoreGiveFromISR(timer_semaphore, NULL); // セマフォを与える
}

bool mrd_timer_setup(int a_duration_ms) {
  timer_waypoint = (unsigned long)(a_duration_ms * (1000 / TIMER_SECTION_US));
  // タイマーの初期化
  mrd_timer_clear();
  // タイマーの設定
  timer_semaphore = xSemaphoreCreateBinary();      // セマフォの作成
  timer = timerBegin(0, 80, true);                 // タイマーの設定(1つ目のタイマーを使用, 分周比80)
  timerAttachInterrupt(timer, &frame_timer, true); // frame_timer関数をタイマーの割り込みに登録
  timerAlarmWrite(timer, TIMER_SECTION_US, true);  // タイマーを1msごとにトリガー
  timerAlarmEnable(timer);                         // タイマーを開始

  return true;
}

int mrd_timer_delay() {
  unsigned long current_count_timer = 0;
  bool flag_loop = true;
  while (flag_loop) {
    if (xSemaphoreTake(timer_semaphore, 0) == pdTRUE) {
      portENTER_CRITICAL(&timer_mux);
      current_count_timer = count_timer; // ハードウェアタイマーの値を読む
      portEXIT_CRITICAL(&timer_mux);
      if (current_count_timer >= timer_waypoint) {
        portENTER_CRITICAL(&timer_mux);
        count_timer = count_timer - timer_waypoint;
        portEXIT_CRITICAL(&timer_mux);
        flag_loop = false;
        break;
      } else {
        delay(1);
      }
    }
  }
  return 0;
}

} // namespace execution
} // namespace core
} // namespace meridian
