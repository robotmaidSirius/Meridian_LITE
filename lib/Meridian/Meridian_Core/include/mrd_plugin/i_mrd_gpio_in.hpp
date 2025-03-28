/**
 * @file i_mrd_gpio_in.hpp
 * @brief MeridianCoreで使用するGPIO入力のインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_GPIO_IN_HPP
#define I_MRD_GPIO_IN_HPP

#include "Meridim90.hpp"

class I_Meridian_GPIO_In {
 public:
  virtual ~I_Meridian_GPIO_In() = default;
  virtual bool read() = 0;

  virtual bool refresh(Meridim90Union &a_meridim) = 0;
};

#endif  // I_MRD_GPIO_IN_HPP
