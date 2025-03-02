/**
 * @file i_mrd_joypad.hpp
 * @brief MeridianCoreで使用するジョイパッドのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_JOYPAD_HPP
#define I_MRD_JOYPAD_HPP

#include "Meridim90.hpp"
#define PAD_GENERALIZE 1 // ジョイパッドの入力値をPS系に一般化する

class I_Meridian_Joypad {
public:
  virtual bool setup() = 0;

  virtual const char *get_name() = 0;
  virtual void set(uint8_t data) = 0;
  virtual uint8_t get() = 0;

  virtual bool refresh(Meridim90Union &a_meridim) = 0;
};

#endif // I_MRD_JOYPAD_HPP
