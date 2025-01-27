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

// ヘッダファイルの読み込み
#include <mrd_communication/i_mrd_conversation.hpp>

// ライブラリ導入
#include <WiFi.h>
#include <WiFiUdp.h>

namespace meridian {
namespace core {
namespace communication {

class MrdConversationWifi : public IMeridianConversation {
private:
  int _open_port = 22224;

public:
  MrdConversationWifi() {}
  const char *type_name() override { return "Wifi"; };

  bool connect(const char *ssid, const char *password, int open_port = 22224) {
    this->_open_port = open_port;
    WiFi.disconnect(true, true); // 新しい接続のためにWiFi接続をリセット
    delay(100);
    WiFi.begin(ssid, password); // Wifiに接続
    int delay_ms = 50;
    int timeout_ms = 10 * (1000);
    int logging_time_ms = (500);

    while (WiFi.status() != WL_CONNECTED) { // https://www.arduino.cc/en/Reference/WiFiStatus 戻り値一覧
      timeout_ms -= delay_ms;
      if (0 == timeout_ms % logging_time_ms) { // 0.5秒ごとに接続状況を出力
        this->a_diag->log(".");
      }
      delay(delay_ms);       // 接続が完了するまでループで待つ
      if (0 >= timeout_ms) { // 10秒でタイムアウト
        this->a_diag->log_error("Wifi init TIMEOUT.");
        return false;
      }
    }
    return this->a_udp.begin(this->_open_port);
  }

  bool setup() override {
    return true;
  }

  bool received(Meridim90 &a_meridim) {
    byte a_meridim_bval[94] = {0};
    int a_len = 90;
    if (this->a_udp.parsePacket() >= a_len) // データの受信バッファ確認
    {
      this->a_udp.read(a_meridim_bval, a_len); // データの受信
      return true;
    }
    return false; // バッファにデータがない
  }
  bool send(Meridim90 &a_meridim) {
    byte a_meridim_bval[94] = {0};
    int a_len = 90;
    for (int i = 0; i < MrdConversationWifi::NUMBER_ALLOWED; i++) {
      if (0 != target[i].port) {
        this->a_udp.beginPacket(target[i].ip, target[i].port); // UDPパケットの開始
        this->a_udp.write(a_meridim_bval, a_len);              // データの書き込み
        this->a_udp.endPacket();                               // UDPパケットの終了
      }
    }
    return true;
  }

public:
  const char *get_ip_address() {
    return WiFi.localIP().toString().c_str();
  }
  bool add_target(const char *ip, uint16_t port) {
    for (int i = 0; i < MrdConversationWifi::NUMBER_ALLOWED; i++) {
      if (0 == target[i].port) {
        target[i].ip.fromString(ip);
        target[i].port = port;
        return true;
      }
    }
    return false;
  }
  bool clear_target() {
    for (int i = 0; i < MrdConversationWifi::NUMBER_ALLOWED; i++) {
      target[i].port = 0;
    }
    return true;
  }
  bool clear_target(int index) {
    if (0 <= index && index < MrdConversationWifi::NUMBER_ALLOWED) {
      target[index].port = 0;
      return true;
    }
    return false;
  }

private:
  WiFiUDP a_udp; // wifi設定
  struct target_send {
    IPAddress ip;
    uint16_t port = 0;
  };

public:
  static const int NUMBER_ALLOWED = 1;
  target_send target[MrdConversationWifi::NUMBER_ALLOWED];
};

} // namespace communication
} // namespace core
} // namespace meridian

#endif // MRD_CONVERSATION_WIFI_HPP
