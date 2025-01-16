/**
 * @file i_mrd_wifi.hpp
 * @brief
 * @version 0.25.1
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_WIFI_HPP
#define I_MRD_WIFI_HPP

#include <stdint.h>

class I_Meridian_WiFi {
public:
  virtual ~I_Meridian_WiFi() = default;
  virtual void connect(const char *ssid, const char *password) = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() = 0;
};

#endif // I_MRD_WIFI_HPP
