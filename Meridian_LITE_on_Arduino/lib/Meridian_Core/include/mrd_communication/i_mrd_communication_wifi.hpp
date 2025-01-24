/**
 * @file i_mrd_plugin_wifi.hpp
 * @brief MeridianCoreで使用するWiFiのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_PLUGIN_WIFI_HPP
#define I_MRD_PLUGIN_WIFI_HPP

#include "i_mrd_communication.hpp"

namespace meridian {
namespace core {
namespace communication {

class I_Meridian_Wifi : public I_Meridian_Communication {
public:
  virtual bool connect(const char *ssid, const char *password) = 0;
  virtual bool disconnect() = 0;
  virtual bool isConnected() = 0;
};

} // namespace communication
} // namespace core
} // namespace meridian

#endif // I_MRD_PLUGIN_WIFI_HPP
