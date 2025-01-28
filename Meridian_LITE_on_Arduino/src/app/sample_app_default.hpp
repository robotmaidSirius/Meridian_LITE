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

#if defined(MODULE_PAD_WIIMOTE)
#include <mrd_module/pad/mrd_module_pad_wiimote.hpp>
#elif defined(MODULE_PAD_KRC5FH)
#include <mrd_module/pad/mrd_module_pad_krc5fh.hpp>
#endif
#if 0
#include <mrd_module/servo/mrd_module_servo_ics.hpp>
#else
#include <mrd_modules/mrd_plugin/i_mrd_plugin_servo.hpp>
#endif

using namespace meridian::modules::plugin;

class sample_app_default {
public:
#if defined(MODULE_PAD_WIIMOTE)
  MrdPadWiimote pad;
#elif defined(MODULE_PAD_KRC5FH)
  MrdPadWiimote pad;
#else
  IMeridianPad pad;
#endif
#if 0
  MrdServoICS servo_left;
  MrdServoICS servo_right;
#else
  IMeridianServo servo_left;
  IMeridianServo servo_right;
#endif

public:
  void setup();
  void loop(Meridim90 &a_meridim90);
};

#endif // __SAMPLE_APP_DEFAULT_HPP__
