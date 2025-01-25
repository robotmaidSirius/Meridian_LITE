#ifndef __MERIDIAN_KEYS_H__
#define __MERIDIAN_KEYS_H__

#include "._system_key.hpp"

// ESP32のIPアドレスを固定する場合は下記の4項目を設定
#ifndef MODE_FIXED_IP
#define MODE_FIXED_IP 0 // IPアドレスを固定するか（0:NO, 1:YES）
#endif
#ifndef FIXED_IP_ADDR
#define FIXED_IP_ADDR "192.168.1.xx" // ESP32のIPアドレスを固定時のESPのIPアドレス
#endif
#ifndef FIXED_IP_GATEWAY
#define FIXED_IP_GATEWAY "192.168.1.xx" // ESP32のIPアドレスを固定時のルーターのゲートウェイ
#endif
#ifndef FIXED_IP_SUBNET
#define FIXED_IP_SUBNET "255.255.255.0" // ESP32のIPアドレスを固定時のサブネット
#endif

#endif // __MERIDIAN_KEYS_H__
