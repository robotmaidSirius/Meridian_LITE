/**
 * @file i_mrd_plugin_joypad.hpp
 * @brief MeridianCoreで使用するジョイパッドのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __I_MRD_PLUGIN_JOYPAD_HPP__
#define __I_MRD_PLUGIN_JOYPAD_HPP__

#include "i_mrd_plugin.hpp"

namespace meridian {
namespace modules {
namespace plugin {

class IMeridianJoypad : public IMeridianPlugin {
public:
  enum hat_t {
    N = 0,
    NE = 1,
    E = 2,
    SE = 3,
    S = 4,
    SW = 5,
    W = 6,
    NW = 7,
    NO_HAT = 8
  };
  struct JoypadInfo {
    uint8_t report_id;
    // Directional buttons
    hat_t hat;
    // Share button
    bool share;
    // Options button
    bool options;
    // PS button
    bool ps;

    // Action Buttons
    bool triangle;
    bool square;
    bool circle;
    bool cross;
    bool fn1;
    bool fn2;
    bool mute;

    // Touch pad/Touch pad button
    //  To use the touch pad button, simply press the touch pad.
    bool touch;
    uint8_t touch_packet_size;
    uint8_t touch_timestamp;

    bool touch_f1_active;
    uint8_t touch_f1_counter;
    uint16_t touch_f1_x;
    uint16_t touch_f1_y;

    bool touch_f2_active;
    uint8_t touch_f2_counter;
    uint16_t touch_f2_x;
    uint16_t touch_f2_y;

    // Touch pad button[Spare]
    bool touch_f1_spare_active;
    uint8_t touch_f1_spare_counter;
    uint16_t touch_f1_spare_x;
    uint16_t touch_f1_spare_y;

    bool touch_f2_spare_active;
    uint8_t touch_f2_spare_counter;
    uint16_t touch_f2_spare_x;
    uint16_t touch_f2_spare_y;

    // Top left button
    bool l1;
    bool l2;
    uint8_t l2_value;
    // Top Right button
    bool r1;
    bool r2;
    uint8_t r2_value;

    // Left stick/L3 button
    //    Press on the stick to use it as the R3 button.
    bool l3;
    uint8_t l3_x;
    uint8_t l3_y;

    // Right stick / R3 button
    //    Press on the stick to use it as the L3 button.
    bool r3;
    uint8_t r3_x;
    uint8_t r3_y;

    // Gyro sensor
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    // Acceleration sensor
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    // Battery level
    uint8_t battery;
    uint8_t battery_level;
    bool connected_usb;
    bool connected_mic;
    bool connected_phone;

    // Timestamp
    uint32_t timestamp;

    // Status
    bool linked;
    const int version = 20250119;
  };

public:
  virtual const char *get_name() { return "None"; };

public:
  JoypadInfo buttons;
  bool isConnected() { return this->buttons.linked; };
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_JOYPAD_HPP__
