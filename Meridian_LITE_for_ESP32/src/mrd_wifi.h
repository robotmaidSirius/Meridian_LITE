#ifndef __MERIDIAN_WIFI_H__
#define __MERIDIAN_WIFI_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"
#include "main.h"

// ライブラリ導入
#include <Dns.h>
#include <Ethernet3.h>
#include <EthernetUdp3.h>
#include <SPI.h>
EthernetUDP udp;

#ifndef SETTINGS_DEFAULT_PIN_ETHERNET_CS
#define SETTINGS_DEFAULT_PIN_ETHERNET_CS 5
#endif
#ifndef SETTINGS_DEFAULT_MAC_ADDRESS
#ifndef ETHERNET_DEFAULT_PIN_RESET
#error SETTINGS_DEFAULT_MAC_ADDRESS が未設定の場合はランダムMACアドレスを設定しますが、#define ETHERNET_DEFAULT_PIN_RESET (GPIO-PIN) で、リセットPINを指定してください。
#endif
#endif
IPAddress _WIFI_SEND_IP; // UDP送信先IPアドレス
bool enable_send = true; // UDP送信を有効にするかどうか

inline byte *toMAC(String mac) {
  static byte mac_str[6];
  char temp[3];
  int i;
  for (i = 0; i < 6; i++) {
    if (mac.length() >= (i * 3 + 2)) {
      temp[0] = mac[i * 3 + 0];
      temp[1] = mac[i * 3 + 1];
      temp[2] = 0x00;
      mac_str[i] = strtol(temp, NULL, 16);
    }
  }
  return (byte *)mac_str;
}
//==================================================================================================
//  Wifi 関連の処理
//==================================================================================================

/// @brief wifiを初期化する.
/// @param a_ssid WifiアクセスポイントのSSID.
/// @param a_pass Wifiアクセスポイントのパスワード.
/// @param a_serial 出力先シリアルの指定.
/// @return 初期化に成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_wifi_init(EthernetUDP &a_udp, const char *a_ssid, const char *a_pass,
                   HardwareSerial &a_serial) {
  bool result = true;
#if defined(SETTINGS_DEFAULT_MAC_ADDRESS)
  byte *mac = toMAC(SETTINGS_DEFAULT_MAC_ADDRESS);
#else
  byte *mac = new byte[6]{0x0E, 0x00, 0x00, 0x00, 0x00, 0x00};
  for (int i = 0; i < 6; i++) {
    mac[i] = random(0, 255);
  }
  mac[0] = (mac[0] | 0x02) & 0xFE; // 1bit目：ユニキャスト ビット 2bit目：ローカル管理アドレス
#endif
#ifdef ETH_HOST_NAME
  Ethernet.setHostname(ETH_HOST_NAME);
#endif
  Ethernet.setCsPin(SETTINGS_DEFAULT_PIN_ETHERNET_CS);
#ifdef ETHERNET_DEFAULT_PIN_RESET
  Ethernet.setRstPin(ETHERNET_DEFAULT_PIN_RESET);
#endif
  Ethernet.init();

  if (MODE_FIXED_IP) { // IPアドレスを固定する場合
    IPAddress ip;
    IPAddress gateway;
    IPAddress subnet;
    ip.fromString(FIXED_IP_ADDR);
    gateway.fromString(FIXED_IP_GATEWAY);
    subnet.fromString(FIXED_IP_SUBNET);
    Ethernet.begin(mac, ip, gateway, subnet);
  } else { // IPアドレスをDHCPで取得する場合
    if (0 != Ethernet.begin(mac)) {
      DNSClient dns;
      dns.begin(Ethernet.dnsServerIP()); // DNSサーバーを開始
      if (1 != dns.getHostByName(WIFI_SEND_IP, _WIFI_SEND_IP)) {
        // Send禁止
        enable_send = false;
      }
      result = true;
    }
  }

  // start UDP
  if (true == result) {
    int ret = udp.begin(UDP_RESV_PORT);
    if (0 == ret) {
      a_serial.println(F("UDP begin failed"));
      result = false;
    } else {
      a_serial.print(F("UDP started on port: "));
      a_serial.println(UDP_RESV_PORT);
    }
  }
  return result;
}

/// @brief 第一引数のMeridim配列にUDP経由でデータを受信, 格納する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するWiFiUDPのインスタンス
/// @return 受信した場合はtrueを, 受信しなかった場合はfalseを返す.
bool mrd_wifi_udp_receive(byte *a_meridim_bval, int a_len, EthernetUDP &a_udp) {
  if (a_udp.parsePacket() >= a_len) {            // データの受信バッファ確認
    if (0 < a_udp.read(a_meridim_bval, a_len)) { // データの受信
      return true;
    }
  }
  return false; // バッファにデータがない
}

/// @brief 第一引数のMeridim配列のデータをUDP経由でWIFI_SEND_IP, UDP_SEND_PORTに送信する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するWiFiUDPのインスタンス
/// @return 送信完了時にtrueを返す.
/// ※WIFI_SEND_IP, UDP_SEND_PORTを関数内で使用.
bool mrd_wifi_udp_send(byte *a_meridim_bval, int a_len, EthernetUDP &a_udp) {
  if (true == enable_send) {
    a_udp.beginPacket(_WIFI_SEND_IP, UDP_SEND_PORT); // UDPパケットの開始
    a_udp.write(a_meridim_bval, a_len);              // データの書き込み
    a_udp.endPacket();                               // UDPパケットの終了
  }
  return true;
}

#endif // __MERIDIAN_WIFI_H__
