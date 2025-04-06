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

#define DEBUG_MERIDIAN_CORE 0

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
#ifdef TEST_CODE_FOR_TIMER
// ハードウェアタイマーとカウンタ用変数の定義
const int TIMER_SECTION_US = 100;              ///! タイマーの分解能(100us)
const double TIMER_HOT_SLEEP_RATE = 0.9;       ///! スリープモードの割合
hw_timer_t *timer = NULL;                      ///! ハードウェアタイマーの設定
uint8_t timer_no = 0;                          ///! タイマー番号
volatile unsigned long count_timer = 0;        ///! フレーム用タイマーのカウントアップ
volatile bool flag_reset_count_timer = false;  ///! フレーム用タイマーのカウントリセット
volatile bool flag_reduce_count_timer = false; ///! フレーム用タイマーのカウントを減少させる

unsigned long timer_waypoint = 10.0 * (1000.0 / TIMER_SECTION_US);                               ///! タイマーの目標値
unsigned long timer_waypoint_keep = (10.0 * TIMER_HOT_SLEEP_RATE) * (1000.0 / TIMER_SECTION_US); ///! スリープモードの目標値

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
}

bool mrd_timer_setup(int a_duration_ms) {
  timer_waypoint = (unsigned long)(a_duration_ms * (1000.0 / TIMER_SECTION_US));
  timer_waypoint_keep = (unsigned long)((a_duration_ms * TIMER_HOT_SLEEP_RATE) * (1000.0 / TIMER_SECTION_US));
  // タイマーの初期化
  mrd_timer_clear();
  // タイマーの設定
  timer = timerBegin(timer_no, getApbFrequency() / 1000000, true); // タイマーの設定(1つ目のタイマーを使用, 分周比80)
  timerAttachInterrupt(timer, &frame_timer, false);                // frame_timer関数をタイマーの割り込みに登録
  timerAlarmWrite(timer, TIMER_SECTION_US, true);                  // トリガー時間の設定
  timerAlarmEnable(timer);                                         // タイマーを開始

  return true;
}

void mrd_timer_delay() {
  unsigned long current_count_timer = 0;
  bool loop_count = false;
  bool flag_loop = true;
  while (flag_loop) {
    current_count_timer = count_timer; // ハードウェアタイマーの値を読む
    if (current_count_timer >= timer_waypoint) {
      flag_reduce_count_timer = true;
      flag_loop = false;
      break;
    } else if (current_count_timer < timer_waypoint_keep) {
      // delay sleep
      delay(1);
    } else {
      // hot sleep
    }
    loop_count = true;
  }
  if (false == loop_count) {
    mrd_timer_clear();
  }
}
#else

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

bool mrd_timer_setup(int a_duration_ms) {
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

  return true;
}

void mrd_timer_delay() {
  static int overflow_count = OVERFLOW_CLEAR_COUNT_MAX;
  unsigned long current_count_timer = 0;
  int skip_counting = 0;
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
      skip_counting++;
      if (current_count_timer < timer_waypoint_keep) {
        // delay sleep
        delay(1);
      } else {
        // hot sleep
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
    log_e("cycle: [BOARDER:%d] %d(skip:%5d)", timer_waypoint, current_count_timer, skip_counting);
  } else {
    log_i("cycle: [BOARDER:%d] %d(skip:%5d)", timer_waypoint, current_count_timer, skip_counting);
#endif
  }
}
#endif

} // namespace execution
} // namespace core
} // namespace meridian
