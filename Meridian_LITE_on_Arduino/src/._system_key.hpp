/**
 * @file ._system_key.hpp
 * @brief Meridian_Console(PCなど)との通信のための設定ファイル
 * @version 1.2.0
 * @date 2025-01-26
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MERIDIAN_SYSTEM_KEYS_HPP__
#define __MERIDIAN_SYSTEM_KEYS_HPP__

// Wifiアクセスポイントの設定
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "xxxxxxxx" /**< アクセスポイントのWIFI_AP_SSID */
#endif

#ifndef WIFI_AP_PASS
#define WIFI_AP_PASS "xxxxxxxx" // アクセスポイントのパスワード
#endif

#ifndef WIFI_SEND_IP
#define WIFI_SEND_IP "192.168.1.xx" // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#endif

#ifndef UDP_SEND_PORT
#define UDP_SEND_PORT 22222 // 送り先のポート番号
#endif

#ifndef UDP_RESV_PORT
#define UDP_RESV_PORT 22224 // このESP32のポート番号
#endif

#endif // __MERIDIAN_KEYS_H__
