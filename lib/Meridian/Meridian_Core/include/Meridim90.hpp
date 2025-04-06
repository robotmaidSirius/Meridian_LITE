/**
 * @file Meridim90.hpp
 * @brief Meridianで定義されているMeridim90型
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MERIDIM90_HPP__
#define __MERIDIM90_HPP__

#include <stdint.h>

namespace meridian {
namespace core {
namespace meridim {

#ifndef MERIDIM90_SIZE
#define MERIDIM90_SIZE (90) /**< Meridimの変数の個数 (デフォルトは90) */
#endif
#ifndef MERIDIM90_SERVO_NUM
#define MERIDIM90_SERVO_NUM (30) /**< 接続するサーボの数 */
#endif
#if (MERIDIM90_SIZE <= (2 + 12 + 1 + 4 + 1 + (MERIDIM90_SERVO_NUM * 2) + 2))
// (Header[2](マスターコマンド+シーケンス) + AHRS[12] + Temp[1] + PAD[4] + Motion[1] + "サーボの数xサイズ倍率" + Footer[2](エラーコード+チェックサム))
#error "MERIDIM90_SERVO_NUM を減らしてください"
#error "Please reduce MERIDIM90_SERVO_NUM"
#endif
/// @brief ユーザー定義用のサイズ
/// @details 全体の個数 - (Header[2](マスターコマンド+シーケンス) + AHRS[12] + Temp[1] + PAD[4] + Motion[1] + Servo[MERIDIM90_SERVO_NUM*2] + Footer[2](エラーコード+チェックサム )
const int MERIDIM90_USER_DATA_SIZE = (MERIDIM90_SIZE - (2 + 12 + 1 + 4 + 1 + (MERIDIM90_SERVO_NUM * 2) + 2));

const int MERIDIM90_BYTE_LEN = (MERIDIM90_SIZE * 2);     ///! Meridim配列のバイト型の長さ (1つのデータが2バイトのため)
const int MERIDIM90_DATA_LEN = (MERIDIM90_BYTE_LEN - 1); /// チェックサムを除いたバイト型の長さ

//! @brief マスターコマンド定義
enum MasterCommand {
  MCMD_TORQUE_ALL_OFF = 0x0000u,         ///! すべてのサーボトルクをオフにする (脱力)
  MCMD_DUMMY_DATA = 0x8000u,             ///! SPI送受信用のダミーデータ判定用
  MCMD_TEST_VALUE = 0x8001u,             ///! テスト用の仮設変数
  MCMD_SENSOR_YAW_CALIB = 0x2712u,       ///! センサの推定ヨー軸を現在値センターとしてリセット
  MCMD_SENSOR_ALL_CALIB = 0x2713u,       ///! センサの3軸について現在値を原点としてリセット
  MCMD_ERR_CLEAR_SERVO_ID = 0x2714u,     ///! 通信エラーのサーボのIDをクリア (MRD_ERR_l)
  MCMD_BOARD_TRANSMIT_ACTIVE = 0x2715u,  ///! ボードが定刻で送信を行うモード(PC側が受信待ち)
  MCMD_BOARD_TRANSMIT_PASSIVE = 0x2716u, ///! ボードが受信を待ち返信するモード(PC側が定刻送信)
  MCMD_FRAMETIMER_RESET = 0x2717u,       ///! フレームタイマーを現在時刻にリセット
  MCMD_BOARD_STOP_DURING = 0x2718u,      ///! ボードの末端処理を[MRD_STOP_FRAMES]ミリ秒止める
  MCMD_EEPROM_ENTER_WRITE = 0x2719u,     ///! EEPROM書き込みモードのスタート
  MCMD_EEPROM_EXIT_WRITE = 0x271Au,      ///! EEPROM書き込みモードの終了
  MCMD_EEPROM_ENTER_READ = 0x271Bu,      ///! EEPROM読み出しモードのスタート
  MCMD_EEPROM_EXIT_READ = 0x271Cu,       ///! EEPROM読み出しモードの終了
  MCMD_SD_CARD_ENTER_WRITE = 0x271Du,    ///! SD_CARD書き込みモードのスタート
  MCMD_SD_CARD_EXIT_WRITE = 0x271Eu,     ///! SD_CARD書き込みモードの終了
  MCMD_SD_CARD_ENTER_READ = 0x271Fu,     ///! SD_CARD読み出しモードのスタート
  MCMD_SD_CARD_EXIT_READ = 0x2720u,      ///! SD_CARD読み出しモードの終了
  MCMD_EEPROM_SAVE_TRIM = 0x2775u,       ///! 現在の姿勢をトリム値としてサーボに書き込む
  MCMD_EEPROM_LOAD_TRIM = 0x2776u,       ///! EEPROMのトリム値をサーボに反映する
  MCMD_NAK = 0x7FFEu,                    ///! コマンド実行の失敗を応答
  MCMD_ACK = 0x7FFFu,                    ///! コマンド実行の成功を応答
};

//! @brief エラービット MRD_ERR_CODEの上位8bit分
enum ErrorBit {
  ERRBIT_ESP_PC = (0b1 << 15),       ///! ESP32 → PC のUDP受信エラー (0:エラーなし、1:エラー検出)
  ERRBIT_PC_ESP = (0b1 << 14),       ///! PC → ESP32 のUDP受信エラー
  ERRBIT_ESP_TSY = (0b1 << 13),      ///! ESP32 → TeensyのSPI受信エラー
  ERRBIT_TSY_ESP = (0b1 << 12),      ///! Teensy → ESP32 のSPI受信エラー
  ERRBIT_BOARD_DELAY = (0b1 << 11),  ///! Teensy or ESP32の処理ディレイ (末端で捕捉)
  ERRBIT_UDP_ESP_SKIP = (0b1 << 10), ///! PC → ESP32 のUDPフレームスキップエラー
  ERRBIT_BOARD_SKIP = (0b1 << 9),    ///! PC → ESP32 → Teensy のフレームスキップエラー (末端で捕捉)
  ERRBIT_PC_SKIP = (0b1 << 8),       ///! Teensy → ESP32 → PC のフレームスキップエラー (末端で捕捉)
  ERRBIT_COMMON = (0b1 << 7),        ///! Teensy → ESP32 → PC のフレームスキップエラー (末端で捕捉)
};

//================================================================================================================
// Meridim90 配列 一覧表
//================================================================================================================
//
// [00]      マスターコマンド デフォルトは90 で配列数も同時に示す
// [01]      シーケンス番号
// [02]-[04] IMU/AHRS:acc＿x,acc＿y,acc＿z    加速度x,y,z
// [05]-[07] IMU/AHRS:gyro＿x,gyro＿y,gyro＿z ジャイロx,y,z
// [08]-[10] IMU/AHRS:mag＿x,mag＿y,mag＿z    磁気コンパスx,y,z
// [11]      IMU/AHRS:temp                   温度
// [12]-[14] IMU/AHRS:DMP ROLL,PITCH,YAW     DMP推定値 ロール,ピッチ,ヨー
// [15]      ボタンデータ1
// [16]      ボタンアナログ1 (Stick Left)
// [17]      ボタンアナログ2 (Stick Right)
// [18]      ボタンアナログ3 (L2,R2 アナログ)
// [19]      モーション設定 (フレーム数)
// [20]      サーボID LO  コマンド
// [21]      サーボID LO  データ値
// [...]     ...
// [48]      サーボID L14 コマンド
// [49]      サーボID L14 データ値
// [50]      サーボID RO  コマンド
// [51]      サーボID RO  データ値
// [...]     ...
// [78]      サーボID R14 コマンド
// [79]      サーボID R14 データ値
// [80]-[MERIDIM90_LEN-3] free (Meridim90では[87]まで)
// [MERIDIM90_LEN-2] ERROR CODE
// [MERIDIM90_LEN-1] チェックサム

struct Meridim90Vector {
  int16_t x;
  int16_t y;
  int16_t z;
};
struct Meridim90RPY {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
};
struct Meridim90Servo {
  uint8_t id;
  uint8_t cmd;
  uint16_t value;
};

struct Meridim90Controller {
  int16_t buttons;  ///! リモコンの基本ボタン値
  int16_t stick_l;  ///! リモコンの左スティックアナログ値
  int16_t stick_r;  ///! リモコンの右スティックアナログ値
  uint8_t analog_l; ///! リモコンのL2ボタンアナログ値
  uint8_t analog_r; ///! リモコンのR2ボタンアナログ値
};
struct Meridim90Input {
  Meridim90Vector accelerator;  ///! 加速度センサ値
  Meridim90Vector gyroscope;    ///! 加速度センサ値
  Meridim90Vector magnetometer; ///! 加速度センサ値
  int16_t temperature;          ///! 温度センサ値
  Meridim90RPY dmp;             ///! DMP推定
  Meridim90Controller control;  ///! リモコンの基本ボタン値
};
struct Meridim90Userdata {
  uint8_t motion_frames;  ///! モーション設定のフレーム数
  uint8_t stop_frames_ms; ///! ボード停止時のフレーム数

  Meridim90Servo servo[MERIDIM90_SERVO_NUM]; ///! サーボのコマンドと値
  int16_t options[MERIDIM90_USER_DATA_SIZE]; ///! ユーザー定義用
};

//! @brief Meridim90の構造体
struct Meridim90 {
  int16_t master_command; ///!	マスターコマンド
  uint16_t sequential;    ///! シーケンス番号
#if 0
  uint16_t sequential;          ///! シーケンス番号
  Meridim90Vector accelerator;  ///! 加速度センサ値
  Meridim90Vector gyroscope;    ///! 加速度センサ値
  Meridim90Vector magnetometer; ///! 加速度センサ値
  int16_t temperature;          ///! 温度センサ値
  Meridim90RPY dmp;             ///! DMP推定
  Meridim90Controller control; ///! リモコンの基本ボタン値
#else
  Meridim90Input input_data; ///! リモコンの基本ボタン値
#endif
#if 0
  uint8_t motion_frames;  ///! モーション設定のフレーム数
  uint8_t stop_frames_ms; ///! ボード停止時のフレーム数

  Meridim90Servo servo[MERIDIM90_SERVO_NUM];   ///! サーボのコマンドと値
  int16_t user_data[MERIDIM90_USER_DATA_SIZE]; ///! ユーザー定義用
#else
  Meridim90Userdata userdata; ///! ユーザー定義用
#endif

  uint16_t err;      ///! ERROR CODE
  uint16_t checksum; ///! CHECK SUM
};

struct Meridim90EX {
  Meridim90 meridim;
};

} // namespace meridim
} // namespace core
} // namespace meridian

using namespace meridian ::core ::meridim;

#endif // __MERIDIM90_HPP__
