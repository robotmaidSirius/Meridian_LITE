/**
 * @file mrd_wifi_esp32.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_WIFI_ESP32_HPP
#define MRD_WIFI_ESP32_HPP

#include <mrd_plugin/i_mrd_wifi.hpp>

class MrdWifiESP32 : public I_Meridian_Wifi {
public:
  MrdWifiESP32() {
  }
  ~MrdWifiESP32() {
  }
  void connect(const char *ssid, const char *password) {}
  void disconnect() {}
  bool isConnected() {}
};

#endif // MRD_WIFI_ESP32_HPP
