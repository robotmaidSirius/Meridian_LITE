/**
 * @file meridian_core_for_arduino.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-29
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MERIDIAN_CORE_FOR_ARDUINO_HPP__
#define __MERIDIAN_CORE_FOR_ARDUINO_HPP__

#include "meridian_core.hpp"
using namespace meridian::core::meridim;

namespace meridian {
namespace core {
namespace execution {

//------------------------------------------------------------------------------------
//  タイムアウト監視用タイマー
//------------------------------------------------------------------------------------

/// @brief 指定されたミリ秒のタイムアウトを監視する. mrd_timeout_resetとセットで使う.
/// @param a_limit タイムアウトまでの時間(ms)
/// @return タイムアウトでtrueを返す.
bool mrd_timeout_check(unsigned long a_limit);

/// @brief タイムアウト監視開始フラグをリセットする. mrd_timeout_checkとセットで使う.
void mrd_timeout_reset();

bool mrd_timer_setup(int a_duration_ms);
void mrd_timer_delay();
void mrd_timer_clear();

} // namespace execution
} // namespace core
} // namespace meridian
using namespace meridian::core::execution;

#endif // __MERIDIAN_CORE_FOR_ARDUINO_HPP__
