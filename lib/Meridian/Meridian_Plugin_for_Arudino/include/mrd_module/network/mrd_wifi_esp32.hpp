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

// ヘッダファイルの読み込み
#include "mrd_wifi_keys.h"
#include <mrd_plugin/i_mrd_wifi.hpp>

// ライブラリ導入
#include <WiFi.h>
#include <WiFiUdp.h>

class MrdWifiESP32 : public I_Meridian_Wifi {
private:
  WiFiUDP _udp; // wifi設定
public:
  MrdWifiESP32() {
  }
  ~MrdWifiESP32() {
  }

public:
  bool setup() override { return true; }
  bool refresh(Meridim90Union &a_meridim) override {
    return false;
  }

  /// @brief wifiを初期化する.
  /// @param a_ssid WifiアクセスポイントのSSID.
  /// @param a_pass Wifiアクセスポイントのパスワード.
  /// @param a_serial 出力先シリアルの指定.
  /// @return 初期化に成功した場合はtrueを, 失敗した場合はfalseを返す.
  bool connect(const char *ssid, const char *password) override {
    static const int LOOP_WAIT_MS = 50;
    static const int LOOP_MAX_MS = (10 * 1000) / LOOP_WAIT_MS;
    this->disconnect();
    delay(100);
    WiFi.begin(ssid, password); // Wifiに接続
    int i = 0;
    while (false == this->isConnected()) {
      i++;
      if (i % 10 == 0) { // 0.5秒ごとに接続状況を出力
        Serial.print(".");
      }
      delay(LOOP_WAIT_MS);   // 接続が完了するまでループで待つ
      if (i > LOOP_MAX_MS) { // タイムアウト
        Serial.println("Wifi init TIMEOUT.");
        return false;
      }
    }
    this->_udp.begin(UDP_RESV_PORT);
    return true;
  }
  bool disconnect() override {
    return WiFi.disconnect(true, true); // WiFi接続をリセット
  }
  bool isConnected() override {
    // https://www.arduino.cc/en/Reference/WiFiStatus 戻り値一覧
    return WiFi.status() == WL_CONNECTED;
  }

  /// @brief 第一引数のMeridim配列のデータをUDP経由でWIFI_SEND_IP, UDP_SEND_PORTに送信する.
  /// @param a_meridim バイト型のMeridim配列
  /// @return 送信完了時にtrueを返す.
  /// ※WIFI_SEND_IP, UDP_SEND_PORTを関数内で使用.
  bool send(Meridim90Union &a_meridim) override {
    this->_udp.beginPacket(WIFI_SEND_IP, UDP_SEND_PORT); // UDPパケットの開始
    this->_udp.write(a_meridim.bval, MRDM_BYTE);         // データの書き込み
    this->_udp.endPacket();                              // UDPパケットの終了
    return true;
  }

  /// @brief 第一引数のMeridim配列にUDP経由でデータを受信, 格納する.
  /// @param a_meridim_bval バイト型のMeridim配列
  /// @param a_len バイト型のMeridim配列の長さ
  /// @param a_udp 使用するWiFiUDPのインスタンス
  /// @return 受信した場合はtrueを, 受信しなかった場合はfalseを返す.
  bool received(Meridim90Union &a_meridim) override {
    if (this->_udp.parsePacket() >= MRDM_BYTE) // データの受信バッファ確認
    {
      this->_udp.read(a_meridim.bval, MRDM_BYTE); // データの受信
      return true;
    }
    return false; // バッファにデータがない
  }
};

#endif // MRD_WIFI_ESP32_HPP
