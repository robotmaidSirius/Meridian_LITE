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

#include <stdint.h>

class I_Meridian_Joypad {
public:
  virtual ~I_Meridian_Joypad() = default;
  virtual void set(uint8_t data) = 0;
  virtual uint8_t get() = 0;
};

#endif // I_MRD_JOYPAD_HPP
