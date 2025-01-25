/**
 * @file mrd_conversation_wifi.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-24
 *
 * @copyright Copyright (c) 2025-.
 *
 */

#ifndef MRD_CONVERSATION_WIFI_HPP
#define MRD_CONVERSATION_WIFI_HPP

#include "mrd_communication/i_mrd_conversation.hpp"

namespace meridian {
namespace communication {

class MRD_Conversation_Wifi : public meridian::core::communication::IMeridianConversation {

public:
  MRD_Conversation_Wifi() {
  };

  bool setup() override {
    return true;
  };

  bool connect(const char *ssid, const char *password) {
    return true;
  };

  bool disconnect() {
    return true;
  };

  bool isConnected() {
    return true;
  };
};

} // namespace communication
} // namespace meridian

#endif // MRD_CONVERSATION_WIFI_HPP
