/**
 * @file i_mrd_plugin_servo.hpp
 * @brief MeridianCoreで使用するサーボのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_PLUGIN_SERVO_HPP
#define I_MRD_PLUGIN_SERVO_HPP

#include "i_mrd_plugin.hpp"

class I_Meridian_Servo : public I_Meridian_Plugin {
public:
  virtual const char *get_name() { return "Unknow"; };

  virtual bool write(int a_id, int value) = 0;
  virtual int read(int a_id) = 0;
};

#endif // I_MRD_PLUGIN_SERVO_HPP
