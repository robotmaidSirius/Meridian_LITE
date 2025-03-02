/**
 * @file i_mrd_ahrs.hpp
 * @brief MeridianCoreで使用するAttitude and heading reference systemのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_AHRS_HPP
#define I_MRD_AHRS_HPP

#include "Meridim90.hpp"

class I_Meridian_AHRS {
public:
  virtual ~I_Meridian_AHRS() = default;
  virtual bool setup() = 0;
  virtual bool begin() = 0;

  virtual bool reset() { return true; };

  virtual bool refresh(Meridim90Union &a_meridim) = 0;
};

#endif // I_MRD_AHRS_HPP
