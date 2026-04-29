#ifndef __MERIDIAN_WIFI_H__
#define __MERIDIAN_WIFI_H__

// ヘッダファイルの読み込み

// ライブラリ導入
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

//==================================================================================================
//  Wifi 関連の処理
//==================================================================================================

/// @brief WiFiを初期化する
/// @param a_ssid WiFiアクセスポイントのSSID
/// @param a_pass WiFiアクセスポイントのパスワード
/// @param receive_port UDP受信に使用するポート番号
/// @param a_serial 出力シリアル
/// @return 成功時はtrue, 失敗時はfalse
bool mrd_wifi_init(const char *a_ssid, const char *a_pass, uint16_t receive_port, HardwareSerial &a_serial);

/// @brief UDP経由でデータを受信しMeridim配列に格納する
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型Meridim配列の長さ
/// @return 受信した場合はtrue, 受信しなかった場合はfalse
bool mrd_wifi_udp_receive(byte *a_meridim_bval, int a_len);

/// @brief Meridim配列データをUDP経由でWIFI_SEND_IP, UDP_SEND_PORTへ送信する
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型Meridim配列の長さ
/// @param host 送信先のホスト名またはIPアドレス
/// @param port 送信先のポート番号
/// @return 成功時はtrue, 失敗時はfalse
/// 内部でWIFI_SEND_IP, UDP_SEND_PORTを使用
bool mrd_wifi_udp_send(byte *a_meridim_bval, int a_len, const char *host, uint16_t port);

#endif // __MERIDIAN_WIFI_H__
