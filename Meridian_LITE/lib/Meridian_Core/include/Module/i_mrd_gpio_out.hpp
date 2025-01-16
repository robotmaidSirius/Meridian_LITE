/**
 * @file i_mrd_gpio_out.hpp
 * @brief
 * @version 0.25.1
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_GPIO_OUT_HPP
#define I_MRD_GPIO_OUT_HPP

class I_Meridian_GPIO_Out {
public:
  virtual ~I_Meridian_GPIO_Out() = default;
  virtual void write(bool state) = 0;
};

#endif // I_MRD_GPIO_OUT_HPP
