#ifndef __MERIDIAN_KEYS_H__
#define __MERIDIAN_KEYS_H__

// Wifiアクセスポイントの設定
#define NETWORK_WIFI_AP_SSID  "xxxxxxxx"     // アクセスポイントのWIFI_AP_SSID
#define NETWORK_WIFI_AP_PASS  "xxxxxxxx"     // アクセスポイントのパスワード
#define NETWORK_WIFI_SEND_IP  "192.168.1.xx" // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define NETWORK_UDP_SEND_PORT 22222          // 送り先のポート番号
#define NETWORK_UDP_RESV_PORT 22224          // このESP32のポート番号

// ESP32のIPアドレスを固定する場合は下記の4項目を設定
#define NETWORK_FIXED_IP         0               // IPアドレスを固定するか（0:NO, 1:YES）
#define NETWORK_FIXED_IP_ADDR    "192.168.1.xx"  // ESP32のIPアドレスを固定時のESPのIPアドレス
#define NETWORK_FIXED_IP_GATEWAY "192.168.1.xx"  // ESP32のIPアドレスを固定時のルーターのゲートウェイ
#define NETWORK_FIXED_IP_SUBNET  "255.255.255.0" // ESP32のIPアドレスを固定時のサブネット

#endif // __MERIDIAN_KEYS_H__
