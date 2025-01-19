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
  static const int PAD_LEN = 5; // リモコン用配列の長さ
  typedef union                 // リモコン値格納用
  {
    short sval[I_Meridian_Joypad::PAD_LEN];        // short型で4個の配列データを持つ
    uint16_t usval[I_Meridian_Joypad::PAD_LEN];    // 上記のunsigned short型
    int8_t bval[I_Meridian_Joypad::PAD_LEN * 2];   // 上記のbyte型
    uint8_t ubval[I_Meridian_Joypad::PAD_LEN * 2]; // 上記のunsigned byte型
    uint64_t ui64val;                              // 上記のunsigned int16型
                                                   // [0]button, [1]pad.stick_L_x:pad.stick_L_y,
                                                   // [2]pad.stick_R_x:pad.stick_R_y, [3]pad.L2_val:pad.R2_val
  } PadUnion;

public:
  virtual bool setup() = 0;

  virtual const char *get_name() = 0;
  virtual void set(uint8_t data) = 0;
  virtual uint8_t get() = 0;

  virtual bool refresh(Meridim90Union &a_meridim) = 0;
};

#endif // I_MRD_JOYPAD_HPP
