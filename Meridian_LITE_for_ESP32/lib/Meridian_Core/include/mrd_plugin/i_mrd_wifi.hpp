/**
 * @file i_mrd_wifi.hpp
 * @brief MeridianCoreで使用するWiFiのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_WIFI_HPP
#define I_MRD_WIFI_HPP

#include "Meridim90.hpp"

class I_Meridian_Wifi {
public:
  virtual ~I_Meridian_Wifi() = default;
  virtual void connect(const char *ssid, const char *password) = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() = 0;

  virtual bool refresh(Meridim90Union &a_meridim) = 0;
};

#endif // I_MRD_WIFI_HPP
