#ifndef __MRD_COMMON_H__
#define __MRD_COMMON_H__

// ヘッダファイルの読み込み
#include "config.h"

// ライブラリ導入
#include <Arduino.h>

enum UartLine { // サーボ系統の列挙型(L,R,C)
  L,            // Left
  R,            // Right
  C             // Center
};

// Meridim配列用の共用体の設定
typedef union {
  short sval[MRDM_LEN + 4];           // short型で90個の配列データを持つ
  unsigned short usval[MRDM_LEN + 2]; // 上記のunsigned short型
  uint8_t bval[+4];                   // byte型で180個の配列データを持つ
  uint8_t ubval[MRDM_BYTE + 4];       // 上記のunsigned byte型
} Meridim90Union;

// サーボ用変数
struct ServoParam {
  // サーボの最大接続 (サーボ送受信のループ処理数)
  int num_max;

  // 各サーボのマウントありなし(config.hで設定)
  int ixl_mount[IXL_MAX]; // L系統
  int ixr_mount[IXR_MAX]; // R系統

  // 各サーボのコード上のインデックスに対し, 実際に呼び出すハードウェアのID番号(config.hで設定)
  int ixl_id[IXL_MAX]; // L系統の実サーボ呼び出しID番号
  int ixr_id[IXR_MAX]; // R系統の実サーボ呼び出しID番号

  // 各サーボの正逆方向補正用配列(config.hで設定)
  int ixl_cw[IXL_MAX]; // L系統
  int ixr_cw[IXR_MAX]; // R系統

  // 各サーボの直立ポーズトリム値(config.hで設定)
  float ixl_trim[IXL_MAX]; // L系統
  float ixr_trim[IXR_MAX]; // R系統

  // 各サーボのベンダーと型番(config.hで設定)
  float ixl_type[IXL_MAX]; // L系統
  float ixr_type[IXR_MAX]; // R系統

  // 各サーボのポジション値(degree)
  float ixl_tgt[IXL_MAX] = {0};      // L系統の目標値
  float ixr_tgt[IXR_MAX] = {0};      // R系統の目標値
  float ixl_tgt_past[IXL_MAX] = {0}; // L系統の前回の値
  float ixr_tgt_past[IXR_MAX] = {0}; // R系統の前回の値

  // サーボのエラーカウンタ配列
  int ixl_err[IXL_MAX] = {0}; // L系統
  int ixr_err[IXR_MAX] = {0}; // R系統

  // サーボのコンディションステータス配列
  uint16_t ixl_stat[IXL_MAX] = {0}; // L系統サーボのコンディションステータス配列
  uint16_t ixr_stat[IXR_MAX] = {0}; // R系統サーボのコンディションステータス配列
};

// エラーカウント用
struct MrdErr {
  int esp_pc = 0;   // PCの受信エラー(ESP32からのUDP)
  int pc_esp = 0;   // ESP32の受信エラー(PCからのUDP)
  int esp_tsy = 0;  // Teensyの受信エラー(ESP32からのSPI)
  int tsy_esp = 0;  // ESP32の受信エラー(TeensyからのSPI)
  int esp_skip = 0; // UDP→ESP受信のカウントの連番スキップ回数
  int tsy_skip = 0; // ESP→Teensy受信のカウントの連番スキップ回数
  int pc_skip = 0;  // PC受信のカウントの連番スキップ回数
};

enum PadType {   // リモコン種の列挙型(NONE, PC, MERIMOTE, BLUERETRO, SBDBT, KRR5FH)
  NONE = 0,      // リモコンなし
  PC = 0,        // PCからのPD入力情報を使用
  MERIMOTE = 1,  // MERIMOTE(未導入)
  BLUERETRO = 2, // BLUERETRO(未導入)
  SBDBT = 3,     // SBDBT(未導入)
  KRR5FH = 4,    // KRR5FH
  WIIMOTE = 5,   // WIIMOTE / WIIMOTE + Nunchuk
  WIIMOTE_C = 6, // WIIMOTE+Classic
};

typedef union // リモコン値格納用
{
  short sval[PAD_LEN];        // short型で4個の配列データを持つ
  uint16_t usval[PAD_LEN];    // 上記のunsigned short型
  int8_t bval[PAD_LEN * 2];   // 上記のbyte型
  uint8_t ubval[PAD_LEN * 2]; // 上記のunsigned byte型
  uint64_t ui64val;           // 上記のunsigned int16型
                              // [0]button, [1]pad.stick_L_x:pad.stick_L_y,
                              // [2]pad.stick_R_x:pad.stick_R_y, [3]pad.L2_val:pad.R2_val
} PadUnion;

// EEPROM読み書き用共用体
typedef union {
  uint8_t bval[EEPROM_SIZE]; // 1バイト単位で540個のデータを持つ
  int16_t saval[3][90];      // short型で3*90個の配列データを持つ
  uint16_t usaval[3][90];    // unsigned short型で3*90個の配列データを持つ
  int16_t sval[270];         // short型で270個のデータを持つ
  uint16_t usval[270];       // unsigned short型で270個のデータを持つ
} UnionEEPROM;

// フラグ用変数
struct MrdFlags {
  bool imuahrs_available = true;        // メインセンサ値を読み取る間, サブスレッドによる書き込みを待機
  bool udp_board_passive = false;       // UDP通信の周期制御がボード主導(false) か, PC主導(true)か.
  bool count_frame_reset = false;       // フレーム管理時計をリセットする
  bool stop_board_during = false;       // ボードの末端処理をmeridim[2]秒, meridim[3]ミリ秒だけ止める.
  bool eeprom_write_mode = false;       // EEPROMへの書き込みモード.
  bool eeprom_read_mode = false;        // EEPROMからの読み込みモード.
  bool eeprom_protect = EEPROM_PROTECT; // EEPROMの書き込みプロテクト.
  bool eeprom_load = EEPROM_LOAD;       // 起動時にEEPROMの内容を読み込む
  bool eeprom_set = EEPROM_SET;         // 起動時にEEPROMに規定値をセット
  bool sdcard_write_mode = false;       // SDCARDへの書き込みモード.
  bool sdcard_read_mode = false;        // SDCARDからの読み込みモード.
  bool wire0_init = false;              // I2C 0系統の初期化合否
  bool wire1_init = false;              // I2C 1系統の初期化合否
  bool bt_busy = false;                 // Bluetoothの受信中フラグ(UDPコンフリクト回避用)
  bool spi_rcvd = true;                 // SPIのデータ受信判定
  bool udp_rcvd = false;                // UDPのデータ受信判定
  bool udp_busy = false;                // UDPスレッドでの受信中フラグ(送信抑制)

  bool udp_receive_mode = MODE_UDP_RECEIVE; // PCからのデータ受信実施(0:OFF, 1:ON, 通常は1)
  bool udp_send_mode = MODE_UDP_SEND;       // PCへのデータ送信実施(0:OFF, 1:ON, 通常は1)
  bool meridim_rcvd = false;                // Meridimが正しく受信できたか.
};

enum ImuAhrsType { // 6軸9軸センサ種の列挙型(NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS)
  NO_IMU = 0,      // IMU/AHRS なし.
  MPU6050_IMU = 1, // MPU6050
  MPU9250_IMU = 2, // MPU9250(未設定)
  BNO055_AHRS = 3  // BNO055
};

#endif // __MRD_COMMON_H__
