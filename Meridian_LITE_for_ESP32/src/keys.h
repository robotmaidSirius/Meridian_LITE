/**
 * @file keys.h
 * @brief Meridian_Console(PCなど)との通信のための設定ファイル
 * @version 1.2.0
 * @date 2025-01-26
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MERIDIAN_KEYS_H__
#define __MERIDIAN_KEYS_H__

////////////////////////////////////////////////////////////////////////
// Wifiアクセスポイントの設定
////////////////////////////////////////////////////////////////////////
#ifndef NETWORK_WIFI_AP_SSID
#define NETWORK_WIFI_AP_SSID "xxxxxxxx" /**< アクセスポイントのWIFI_AP_SSID */
#endif

#ifndef NETWORK_WIFI_AP_PASS
#define NETWORK_WIFI_AP_PASS "xxxxxxxx" /**< アクセスポイントのパスワード */
#endif

#ifndef NETWORK_WIFI_SEND_IP
#define NETWORK_WIFI_SEND_IP "192.168.1.xx" /**< 送り先のPCのIPアドレス（PCのIPアドレスを調べておく） */
#endif

#ifndef NETWORK_UDP_SEND_PORT
#define NETWORK_UDP_SEND_PORT 22222 /**< 送り先のポート番号 */
#endif

#ifndef NETWORK_UDP_RESV_PORT
#define NETWORK_UDP_RESV_PORT 22224 /**< このESP32のポート番号 */
#endif

////////////////////////////////////////////////////////////////////////
// ESP32のIPアドレスを固定する場合は下記の4項目を設定
////////////////////////////////////////////////////////////////////////
#ifndef NETWORK_MODE_FIXED_IP
#define NETWORK_MODE_FIXED_IP 0 /**< IPアドレスを固定するか（0:NO, 1:YES） */
#endif
#ifndef NETWORK_FIXED_IP_ADDR
#define NETWORK_FIXED_IP_ADDR "192.168.1.xx" /**< ESP32のIPアドレスを固定時のESPのIPアドレス */
#endif
#ifndef NETWORK_FIXED_IP_GATEWAY
#define NETWORK_FIXED_IP_GATEWAY "192.168.1.xx" /**< ESP32のIPアドレスを固定時のルーターのゲートウェイ */
#endif
#ifndef NETWORK_FIXED_IP_SUBNET
#define NETWORK_FIXED_IP_SUBNET "255.255.255.0" /**< ESP32のIPアドレスを固定時のサブネット */
#endif

#endif // __MERIDIAN_KEYS_H__
