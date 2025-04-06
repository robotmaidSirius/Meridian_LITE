/**
 * @file sample_app_default.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-28
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __SAMPLE_APP_DEFAULT_HPP__
#define __SAMPLE_APP_DEFAULT_HPP__

#define APP_SERVO_BAUDRATE_L 1250000 // L系統のICSサーボの通信速度bps
#define APP_SERVO_BAUDRATE_R 1250000 // R系統のICSサーボの通信速度bps
#define APP_SERVO_TIMEOUT_L  2       // L系統のICS返信待ちのタイムアウト時間
#define APP_SERVO_TIMEOUT_R  2       // R系統のICS返信待ちのタイムアウト時間

#if defined(MODULE_PAD_WIIMOTE)
#include <mrd_module/pad/mrd_module_pad_wiimote.hpp>
#elif defined(MODULE_PAD_KRC5FH)
#include <mrd_module/pad/mrd_module_pad_krc5fh.hpp>
#else
#include <mrd_module/mrd_plugin/i_mrd_plugin_pad.hpp>
#endif
#include <mrd_module/servo/mrd_module_servo_ics.hpp>

using namespace meridian::modules::plugin;

class SampleAppDefault {

public:
  SampleAppDefault();

#if defined(MODULE_PAD_WIIMOTE)
  MrdPadWiimote pad;
#elif defined(MODULE_PAD_KRC5FH)
  MrdPadKRC5FH pad;
#else
  IMeridianPad pad;
#endif
  MrdServoICS servo_left;
  MrdServoICS servo_right;

public:
  void setup();
  void loop(Meridim90 &a_meridim90);
};

#endif // __SAMPLE_APP_DEFAULT_HPP__
