#ifndef __MERIDIAN_WIFI_H__
#define __MERIDIAN_WIFI_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"
#include "main.h"

// ライブラリ導入
#include <WiFi.h>
#include <WiFiUdp.h>

namespace meridian {
namespace core {
namespace communication {

//==================================================================================================
//  Wifi 関連の処理
//==================================================================================================
class MrdConversation {
public:
  MrdConversation(const char *a_send_ip, uint16_t a_send_port, uint16_t a_receive_port) {
    this->_send_ip.fromString(a_send_ip);
    this->_send_port = a_send_port;
    this->_receive_port = a_receive_port;
  }

public:
  /// @brief wifiを初期化する.
  /// @param a_ssid WifiアクセスポイントのSSID.
  /// @param a_pass Wifiアクセスポイントのパスワード.
  /// @param a_serial 出力先シリアルの指定.
  /// @return 初期化に成功した場合はtrueを, 失敗した場合はfalseを返す.
  bool init(const char *a_ssid, const char *a_pass, HardwareSerial &a_serial) {
    WiFi.disconnect(true, true); // 新しい接続のためにWiFi接続をリセット
    delay(100);
    WiFi.begin(a_ssid, a_pass); // Wifiに接続
    int i = 0;
    while (WiFi.status() != WL_CONNECTED) { // https://www.arduino.cc/en/Reference/WiFiStatus 戻り値一覧
      i++;
      if (i % 10 == 0) { // 0.5秒ごとに接続状況を出力
        a_serial.print(".");
      }
      delay(50);     // 接続が完了するまでループで待つ
      if (i > 200) { // 10秒でタイムアウト
        a_serial.println("Wifi init TIMEOUT.");
        return false;
      }
    }
    this->_udp.begin(this->_receive_port);
    return true;
  }

  /// @brief 第一引数のMeridim配列にUDP経由でデータを受信, 格納する.
  /// @param a_meridim_bval バイト型のMeridim配列
  /// @param a_len バイト型のMeridim配列の長さ
  /// @param a_udp 使用するWiFiUDPのインスタンス
  /// @return 受信した場合はtrueを, 受信しなかった場合はfalseを返す.
  bool udp_receive(byte *a_meridim_bval, int a_len) {
    if (this->_udp.parsePacket() >= a_len) // データの受信バッファ確認
    {
      this->_udp.read(a_meridim_bval, a_len); // データの受信
      return true;
    }
    return false; // バッファにデータがない
  }

  /// @brief 第一引数のMeridim配列のデータをUDP経由でWIFI_SEND_IP, UDP_SEND_PORTに送信する.
  /// @param a_meridim_bval バイト型のMeridim配列
  /// @param a_len バイト型のMeridim配列の長さ
  /// @param a_udp 使用するWiFiUDPのインスタンス
  /// @return 送信完了時にtrueを返す.
  /// ※WIFI_SEND_IP, UDP_SEND_PORTを関数内で使用.
  bool udp_send(byte *a_meridim_bval, int a_len) {
    this->_udp.beginPacket(this->_send_ip, this->_send_port); // UDPパケットの開始
    this->_udp.write(a_meridim_bval, a_len);                  // データの書き込み
    this->_udp.endPacket();                                   // UDPパケットの終了
    return true;
  }

private:
  uint16_t _receive_port = 22224; /// 受信ポート番号
  IPAddress _send_ip;             /// 送信先IPアドレス
  uint16_t _send_port = 22222;    /// 送信先ポート番号
  WiFiUDP _udp;                   /// wifi設定
};

} // namespace communication
} // namespace core
} // namespace meridian

#endif // __MERIDIAN_WIFI_H__
