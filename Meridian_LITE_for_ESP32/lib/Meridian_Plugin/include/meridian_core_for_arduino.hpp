/**
 * @file meridian_core_for_arduino.hpp
 * @brief Arduino用Meridianのコアライブラリ
 * @version 1.2.0
 * @date 2025-03-06
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __MERIDIAN_CORE_FOR_ARDUINO_HPP__
#define __MERIDIAN_CORE_FOR_ARDUINO_HPP__

namespace meridian {
namespace core {
namespace execution {

//------------------------------------------------------------------------------------
//  タイムアウト監視用タイマー
//------------------------------------------------------------------------------------

/// @brief タイムアウト監視用のタイマーを設定する.
void mrd_timer_setup(int a_duration_ms);
/// @brief 所定の時間が経過するまで待機する.
void mrd_timer_delay();
/// @brief タイムアウト監視開始フラグをリセットする. mrd_timeout_checkとセットで使う.
void mrd_timer_clear();

} // namespace execution
} // namespace core
} // namespace meridian
using namespace meridian::core::execution;

#endif
