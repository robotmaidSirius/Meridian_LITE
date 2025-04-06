#ifndef __MERIDIAN_LITE_MAIN__
#define __MERIDIAN_LITE_MAIN__

#define VERSION "Meridian_LITE_v1.1.1_2024_08.18" // バージョン表示

#include <board/meridian_board_lite.hpp>

// ヘッダファイルの読み込み
#include "keys.h"

// ライブラリ導入
#include <WiFi.h>

// ライブラリ導入
#include <Meridian.h> // Meridianのライブラリ導入

#include "mrd_communication/mrd_conversation_wifi.hpp"
#include "mrd_module/filesystem/mrd_module_eeprom.hpp"
#include "mrd_module/filesystem/mrd_module_sd.hpp"
#include "mrd_module/servo/mrd_module_servo_ics.hpp"

// ライブラリ導入
#include <Arduino.h>

// ヘッダファイルの読み込み
MERIDIANFLOW::Meridian mrd;
//-------------------------------------------------------------------------
//  各種設定
//-------------------------------------------------------------------------

// Meridimの基本設定
#define MRDM_LEN 90 // Meridim配列の長さ設定(デフォルトは90)

// 各種ハードウェアのマウント有無
#define MOUNT_PAD KRR5FH // ジョイパッドの搭載 PC, MERIMOTE, BLUERETRO, KRR5FH, WIIMOTE

// JOYPAD関連設定
#define PAD_INTERVAL 10 // JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)

//-------------------------------------------------------------------------
//  Meridim90 配列アクセス対応キー
//-------------------------------------------------------------------------
#define MRD_PAD_BUTTONS   15 // リモコンの基本ボタン値
#define MRD_PAD_STICK_L   16 // リモコンの左スティックアナログ値
#define MRD_PAD_STICK_R   17 // リモコンの右スティックアナログ値
#define MRD_PAD_L2R2VAL   18 // リモコンのL2R2ボタンアナログ値
#define MRD_SEQ           1  // シーケンス番号
#define MRD_MOTION_FRAMES 19 // モーション設定のフレーム数
#define MRD_STOP_FRAMES   19 // ボード停止時のフレーム数(MCMD_BOARD_STOP_DURINGで指定)
// Meridimの基本設定
#define FRAME_DURATION 10 // 1フレームあたりの単位時間(単位ms)

// 動作モード
#define MODE_UDP_RECEIVE 1 // PCからのデータ受信 (0:OFF, 1:ON, 通常は1)
#define MODE_UDP_SEND    1 // PCへのデータ送信 (0:OFF, 1:ON, 通常は1)

// 各サーボ系統の最大サーボマウント数
#define IXL_MAX 15 // L系統の最大サーボ数. 標準は15.
#define IXR_MAX 15 // R系統の最大サーボ数. 標準は15.

// EEPROMの設定
#define EEPROM_SET  0 // 起動時にEEPROMにconfig.hの内容をセット(mrd_set_eeprom)
#define EEPROM_LOAD 0 // 起動時にEEPROMの内容を諸設定にロードする(未導入)

//-------------------------------------------------------------------------
//  Meridim90 配列アクセス対応キー
//-------------------------------------------------------------------------
#define MRD_MASTER 0 // マスターコマンド

//================================================================================================================
//================================================================================================================
// シリアルモニタリング
#define MONITOR_ERR_ALL           0    // 全経路の受信エラー率を表示
#define MONITOR_SEQ               0    // シリアルモニタでシーケンス番号チェックを表示(0:OFF, 1:ON)
#define MONITOR_SUPPRESS_DURATION 8000 // 起動直後のタイムアウトメッセージ抑制時間(単位ms)

// Wifiの設定(SSID,パスワード等は別途keys.hで指定)
#define UDP_TIMEOUT 4 // UDPの待受タイムアウト (単位ms,推奨値0)

// 動作モード
#define MODE_ESP32_STDALONE 0 // ESP32をボードに挿さず動作確認 (0:NO, 1:YES)
//------------------------------------------------------------------------------------
//  変数
//------------------------------------------------------------------------------------
// システム用の変数
const int MRDM_BYTE = MRDM_LEN * 2;    // Meridim配列のバイト型の長さ
const int MRD_ERR = MRDM_LEN - 2;      // エラーフラグの格納場所(配列の末尾から2つめ)
const int MRD_ERR_u = MRD_ERR * 2 + 1; // エラーフラグの格納場所(上位8ビット)
const int MRD_ERR_l = MRD_ERR * 2;     // エラーフラグの格納場所(下位8ビット)
const int MRD_CKSM = MRDM_LEN - 1;     // チェックサムの格納場所(配列の末尾)
const int PAD_LEN = 5;                 // リモコン用配列の長さ
//------------------------------------------------------------------------------------
//  列挙型
//------------------------------------------------------------------------------------

enum UartLine { // サーボ系統の列挙型(L,R,C)
  L,            // Left
  R,            // Right
  C             // Center
};

enum ServoType { // サーボプロトコルのタイプ
  NOSERVO = 0,   // サーボなし
  PWM_S = 1,     // Single PWM (WIP)
  PCA9685 = 11,  // I2C_PCA9685 to PWM (WIP)
  FTBRSX = 21,   // FUTABA_RSxTTL (WIP)
  DXL1 = 31,     // DYNAMIXEL 1.0 (WIP)
  DXL2 = 32,     // DYNAMIXEL 2.0 (WIP)
  KOICS3 = 43,   // KONDO_ICS 3.5 / 3.6
  KOPMX = 44,    // KONDO_PMX (WIP)
  JRXBUS = 51,   // JRPROPO_XBUS (WIP)
  FTCSTS = 61,   // FEETECH_STS (WIP)
  FTCSCS = 62    // FEETECH_SCS (WIP)
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

enum PadButton {  // リモコンボタンの列挙型
  PAD_SELECT = 1, // Select
  PAD_HOME = 2,   // HOME
  PAD_L3 = 2,     // L3
  PAD_R3 = 4,     // L4
  PAD_START = 8,  // Start
  PAD_UP = 16,    // 十字上
  PAD_RIGHT = 32, // 十字右
  PAD_DOWN = 64,  // 十字下
  PAD_LEFT = 128, // 十字左
  PAD_L2 = 256,   // L2
  PAD_R2 = 512,   // R2
  PAD_L1 = 1024,  // L1
  PAD_R1 = 2048,  // R1
  PAD_bU = 4096,  // △ 上
  PAD_bR = 8192,  // o 右
  PAD_bD = 16384, // x 下
  PAD_bL = 32768  // ◻︎ 左
};

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

// Meridim配列用の共用体の設定
typedef union {
  short sval[MRDM_LEN + 4];           // short型で90個の配列データを持つ
  unsigned short usval[MRDM_LEN + 2]; // 上記のunsigned short型
  uint8_t bval[MRDM_LEN + 4];         // byte型で180個の配列データを持つ
  uint8_t ubval[MRDM_BYTE + 4];       // 上記のunsigned byte型
} Meridim90Union;

// フラグ用変数
struct MrdFlags {
  bool imuahrs_available = true;            // メインセンサ値を読み取る間, サブスレッドによる書き込みを待機
  bool udp_board_passive = false;           // UDP通信の周期制御がボード主導(false) か, PC主導(true)か.
  bool count_frame_reset = false;           // フレーム管理時計をリセットする
  bool stop_board_during = false;           // ボードの末端処理をmeridim[2]秒, meridim[3]ミリ秒だけ止める.
  bool eeprom_write_mode = false;           // EEPROMへの書き込みモード.
  bool eeprom_read_mode = false;            // EEPROMからの読み込みモード.
  bool eeprom_load = EEPROM_LOAD;           // 起動時にEEPROMの内容を読み込む
  bool eeprom_set = EEPROM_SET;             // 起動時にEEPROMに規定値をセット
  bool sdcard_write_mode = false;           // SDCARDへの書き込みモード.
  bool sdcard_read_mode = false;            // SDCARDからの読み込みモード.
  bool wire0_init = false;                  // I2C 0系統の初期化合否
  bool wire1_init = false;                  // I2C 1系統の初期化合否
  bool bt_busy = false;                     // Bluetoothの受信中フラグ(UDPコンフリクト回避用)
  bool spi_rcvd = true;                     // SPIのデータ受信判定
  bool udp_rcvd = false;                    // UDPのデータ受信判定
  bool udp_busy = false;                    // UDPスレッドでの受信中フラグ(送信抑制)
  bool udp_receive_mode = MODE_UDP_RECEIVE; // PCからのデータ受信実施(0:OFF, 1:ON, 通常は1)
  bool udp_send_mode = MODE_UDP_SEND;       // PCへのデータ送信実施(0:OFF, 1:ON, 通常は1)
  bool meridim_rcvd = false;                // Meridimが正しく受信できたか.
};

// シーケンス番号理用の変数
struct MrdSq {
  int s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
  int r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};

// タイマー管理用の変数
struct MrdTimer {
  long frame_ms = FRAME_DURATION;                                   // 1フレームあたりの単位時間(ms)
  int count_loop = 0;                                               // サイン計算用の循環カウンタ
  int count_loop_dlt = 2;                                           // サイン計算用の循環カウンタを1フレームにいくつ進めるか
  int count_loop_max = 359999;                                      // 循環カウンタの最大値
  unsigned long count_frame = 0;                                    // メインフレームのカウント
  int pad_interval = (PAD_INTERVAL - 1 > 0) ? PAD_INTERVAL - 1 : 1; // パッドの問い合わせ待機時間
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
// リモコンのアナログ入力データ
struct PadValue {
  unsigned short stick_R = 0;
  int stick_R_x = 0;
  int stick_R_y = 0;
  unsigned short stick_L = 0;
  int stick_L_x = 0;
  int stick_L_y = 0;
  unsigned short stick_L2R2V = 0;
  int R2_val = 0;
  int L2_val = 0;
};

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

//================================================================================================================
//  シリアルモニタ表示用の関数
//================================================================================================================

//================================================================================================================
//  関数各種
//================================================================================================================

//================================================================================================================

int IXL_MT[IXL_MAX] = {
    // L系統のマウント状態
    43, // [00]頭ヨー
    43, // [01]左肩ピッチ
    43, // [02]左肩ロール
    43, // [03]左肘ヨー
    43, // [04]左肘ピッチ
    43, // [05]左股ヨー
    43, // [06]左股ロール
    43, // [07]左股ピッチ
    43, // [08]左膝ピッチ
    43, // [09]左足首ピッチ
    43, // [10]左足首ロール
    0,  // [11]追加サーボ用
    0,  // [12]追加サーボ用
    0,  // [13]追加サーボ用
    0   // [14]追加サーボ用
};

// R系統のサーボのマウントの設定
// 00: NOSERVO (マウントなし),            01: PWM_S1 (Single PWM)[WIP]
// 11: PCA9685 (I2C_PCA9685toPWM)[WIP], 21: FTBRSX (FUTABA_RSxTTL)[WIP]
// 31: DXL1 (DYNAMIXEL 1.0)[WIP],       32: DXL2 (DYNAMIXEL 2.0)[WIP]
// 43: KOICS3 (KONDO_ICS 3.5 / 3.6),    44: KOPMX (KONDO_PMX)[WIP]
// 51: JRXBUS (JRPROPO_XBUS)[WIP]
// 61: FTCSTS (FEETECH_STS)[WIP],       62: FTCSCS (FEETECH_SCS)[WIP]
int IXR_MT[IXR_MAX] = {
    // R系統のマウント状態
    43, // [00]腰ヨー
    43, // [01]右肩ピッチ
    43, // [02]右肩ロール
    43, // [03]右肘ヨー
    43, // [04]右肘ピッチ
    43, // [05]右股ヨー
    43, // [06]右股ロール
    43, // [07]右股ピッチ
    43, // [08]右膝ピッチ
    43, // [09]右足首ピッチ
    43, // [10]右足首ロール
    0,  // [11]追加サーボ用
    0,  // [12]追加サーボ用
    0,  // [13]追加サーボ用
    0   // [14]追加サーボ用
};

// L系統のコード上のサーボIndexに対し, 実際に呼び出すハードウェアのID番号
int IXL_ID[IXL_MAX] = {
    0,  // [00]頭ヨー
    1,  // [01]左肩ピッチ
    2,  // [02]左肩ロール
    3,  // [03]左肘ヨー
    4,  // [04]左肘ピッチ
    5,  // [05]左股ヨー
    6,  // [06]左股ロール
    7,  // [07]左股ピッチ
    8,  // [08]左膝ピッチ
    9,  // [09]左足首ピッチ
    10, // [10]左足首ロール
    11, // [11]追加サーボ用
    12, // [12]追加サーボ用
    13, // [13]追加サーボ用
    14  // [14]追加サーボ用
};

// R系統のコード上のサーボIndexに対し, 実際に呼び出すハードウェアのID番号
int IXR_ID[IXR_MAX] = {
    0,  // [00]腰ヨー
    1,  // [01]右肩ピッチ
    2,  // [02]右肩ロール
    3,  // [03]右肘ヨー
    4,  // [04]右肘ピッチ
    5,  // [05]右股ヨー
    6,  // [06]右股ロール
    7,  // [07]右股ピッチ
    8,  // [08]右膝ピッチ
    9,  // [09]右足首ピッチ
    10, // [10]右足首ロール
    11, // [11]追加サーボ用
    12, // [12]追加サーボ用
    13, // [13]追加サーボ用
    14  // [14]追加サーボ用
};

// L系統のサーボ回転方向補正(1:変更なし, -1:逆転)
int IXL_CW[IXL_MAX] = {
    1, // [00]頭ヨー
    1, // [01]左肩ピッチ
    1, // [02]左肩ロール
    1, // [03]左肘ヨー
    1, // [04]左肘ピッチ
    1, // [05]左股ヨー
    1, // [06]左股ロール
    1, // [07]左股ピッチ
    1, // [08]左膝ピッチ
    1, // [09]左足首ピッチ
    1, // [10]左足首ロール
    1, // [11]追加サーボ用
    1, // [12]追加サーボ用
    1, // [13]追加サーボ用
    1  // [14]追加サーボ用
};

// R系統のサーボ回転方向補正(1:変更なし, -1:逆転)
int IXR_CW[IXR_MAX] = {
    // R系統の正転逆転
    1, // [00]腰ヨー
    1, // [01]右肩ピッチ
    1, // [02]右肩ロール
    1, // [03]右肘ヨー
    1, // [04]右肘ピッチ
    1, // [05]右股ヨー
    1, // [06]右股ロール
    1, // [07]右股ピッチ
    1, // [08]右膝ピッチ
    1, // [09]右足首ピッチ
    1, // [10]右足首ロール
    1, // [11]追加サーボ用
    1, // [12]追加サーボ用
    1, // [13]追加サーボ用
    1  // [14]追加サーボ用
};

// L系統のトリム値(degree)
float IDL_TRIM[IXL_MAX] = {
    0,      // [00]頭ヨー
    -2.36,  // [01]左肩ピッチ
    -91.13, // [02]左肩ロール
    0,      // [03]左肘ヨー
    89.98,  // [04]左肘ピッチ
    0,      // [05]左股ヨー
    0,      // [06]左股ロール
    -1.35,  // [07]左股ピッチ
    -58.05, // [08]左膝ピッチ
    -20.25, // [09]左足首ピッチ
    -0.68,  // [10]左足首ロール
    0,      // [11]追加サーボ用
    0,      // [12]追加サーボ用
    0,      // [13]追加サーボ用
    0       // [14]追加サーボ用
};

// R系統のトリム値(degree)
float IDR_TRIM[IXR_MAX] = {
    0,      // [00]腰ヨー
    0,      // [01]右肩ピッチ
    -89.44, // [02]右肩ロール
    0,      // [03]右肘ヨー
    89.98,  // [04]右肘ピッチ
    0,      // [05]右股ヨー
    1.69,   // [06]右股ロール
    -3.38,  // [07]右股ピッチ
    -57.38, // [08]右膝ピッチ
    -20.25, // [09]右足首ピッチ
    -2.36,  // [10]右足首ロール
    0,      // [11]追加サーボ用
    0,      // [12]追加サーボ用
    0,      // [13]追加サーボ用
    0,      // [14]追加サーボ用
};

//================================================================================================================
//  SETUP
//================================================================================================================
MrdEEPROM _eeprom(540);
MrdConversationWifi _comm;
MrdServoICS *_servo_l;
MrdServoICS *_servo_r;
MrdErr err;
TaskHandle_t thp[4];                // マルチスレッドのタスクハンドル格納用
Meridim90Union s_udp_meridim;       // Meridim配列データ送信用(short型, センサや角度は100倍値)
Meridim90Union r_udp_meridim;       // Meridim配列データ受信用
Meridim90Union s_udp_meridim_dummy; // SPI送信ダミー用
Meridim90 meridim90;                // Meridim配列データの格納用
MrdFlags flg;
MrdSq mrdsq;
MrdTimer tmr;
PadUnion pad_array = {0}; // pad値の格納用配列
PadUnion pad_i2c = {0};   // pad値のi2c送受信用配列

PadValue pad_analog;
ServoParam sv;

// 予約用
/// @brief Master Commandの第1群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列を渡す.
/// @param a_flg_exe Meridimの受信成功判定フラグを渡す.
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_1(Meridim90Union a_meridim) {
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:MCMD_ERR_CLEAR_SERVO_ID (10004) 通信エラーサーボIDのクリア
  if (a_meridim.sval[MRD_MASTER] == MCMD_ERR_CLEAR_SERVO_ID) {
    r_udp_meridim.bval[MRD_ERR_l] = 0;
    s_udp_meridim.bval[MRD_ERR_l] = 0;
    for (int i = 0; i < IXL_MAX; i++) {
      sv.ixl_err[i] = 0;
    }
    for (int i = 0; i < IXR_MAX; i++) {
      sv.ixr_err[i] = 0;
    }
    Serial.println("Servo Error ID reset.");
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信の通信周期制御をボード側主導に(デフォルト)
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_ACTIVE) {
    flg.udp_board_passive = false; // UDP送信をアクティブモードに
    flg.count_frame_reset = true;  // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_EEPROM_ENTER_WRITE (10009) EEPROMの書き込みモードスタート
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_ENTER_WRITE) {
    flg.eeprom_write_mode = true; // 書き込みモードのフラグをセット
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }
  return false;
}

/// @brief Master Commandの第2群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列を渡す.
/// @param a_flg_exe Meridimの受信成功判定フラグを渡す.
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_2(Meridim90Union a_meridim) {
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:[0] 全サーボ脱力
  if (a_meridim.sval[MRD_MASTER] == 0) {
    _servo_l->all_off();
    _servo_r->all_off();
    return true;
  }

  // コマンド:[1] サーボオン 通常動作

  // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信の通信周期制御をPC側主導に(SSH的な動作)
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_PASSIVE) {
    flg.udp_board_passive = true; // UDP送信をパッシブモードに
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_FRAMETIMER_RESET) (10007) フレームカウンタを現在時刻にリセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_FRAMETIMER_RESET) {
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_BOARD_STOP_DURING (10008) ボードの末端処理を指定時間だけ止める.
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_STOP_DURING) {
    flg.stop_board_during = true; // ボードの処理停止フラグをセット
    // ボードの末端処理をmeridim[2]ミリ秒だけ止める.
    Serial.print("Stop ESP32's processing during ");
    Serial.print(int(a_meridim.sval[MRD_STOP_FRAMES]));
    Serial.println(" ms.");
    for (int i = 0; i < int(a_meridim.sval[MRD_STOP_FRAMES]); i++) {
      delay(1);
    }
    flg.stop_board_during = false; // ボードの処理停止フラグをクリア
    flg.count_frame_reset = true;  // フレームの管理時計をリセットフラグをセット
    return true;
  }
  return false;
}

/// @brief 配列の中で0以外が入っている最大のIndexを求める.
/// @param a_arr 配列
/// @param a_size 配列の長さ
/// @return 0以外が入っている最大のIndex. すべて0の場合は1を反す.
int mrd_max_used_index(const int a_arr[], int a_size) {
  int max_index_tmp = 0;
  for (int i = 0; i < a_size; ++i) {
    if (a_arr[i] != 0) {
      max_index_tmp = i;
    }
  }
  return max_index_tmp + 1;
}

void test_setup() {
  // サーボ値の初期設定
  sv.num_max = max(mrd_max_used_index(IXL_MT, IXL_MAX), mrd_max_used_index(IXR_MT, IXR_MAX)); // サーボ処理回数

  for (int i = 0; i <= sv.num_max; i++) { // configで設定した値を反映させる
    sv.ixl_mount[i] = IXL_MT[i];
    sv.ixr_mount[i] = IXR_MT[i];
    sv.ixl_id[i] = IXL_ID[i];
    sv.ixr_id[i] = IXR_ID[i];
    sv.ixl_cw[i] = IXL_CW[i];
    sv.ixr_cw[i] = IXR_CW[i];
    sv.ixl_trim[i] = IDL_TRIM[i];
    sv.ixr_trim[i] = IDR_TRIM[i];
  }

  // EEPROMの開始, ダンプ表示
  MrdEEPROM::UnionEEPROM array_tmp = {0};
  for (int i = 0; i < 15; i++) {
    // 各サーボのマウントありなし(0:サーボなし, +:サーボあり順転, -:サーボあり逆転)
    // 例: IXL_MT[20] = -21; → FUTABA_RSxTTLサーボを逆転設定でマウント
    array_tmp.saval[0][20 + i * 2] = short(sv.ixl_mount[i] * sv.ixl_cw[i]);
    array_tmp.saval[0][50 + i * 2] = short(sv.ixr_mount[i] * sv.ixr_cw[i]);
    // 各サーボの直立デフォルト値 degree
    array_tmp.saval[1][21 + i * 2] = mrd.float2HfShort(sv.ixl_trim[i]);
    array_tmp.saval[1][51 + i * 2] = mrd.float2HfShort(sv.ixr_trim[i]);
  }
  _eeprom.eeprom_setup = array_tmp; // EEPROMの設定値をセット

  mrd_timer_setup(10); // タイマーの設定
}
//------------------------------------------------------------------------------------
//  meriput / meridimへのデータ書き込み
//------------------------------------------------------------------------------------

//================================================================================================================
// MAIN LOOP
//================================================================================================================
void test_loop() {

  // @[2-1] UDPの受信待ち受けループ
  if (flg.udp_receive_mode) { // UDPの受信実施フラグの確認(モード確認)
    unsigned long start_tmp = millis();
    flg.udp_busy = true;  // UDP使用中フラグをアゲる
    flg.udp_rcvd = false; // UDP受信完了フラグをサゲる
    while (!flg.udp_rcvd) {
      flg.udp_rcvd = true; // UDP受信完了フラグをアゲる

      // タイムアウト抜け処理
      unsigned long current_tmp = millis();
      if (current_tmp - start_tmp >= UDP_TIMEOUT) {
        if (millis() > MONITOR_SUPPRESS_DURATION) { // 起動直後はエラー表示を抑制
          Serial.println("UDP timeout");
        }
        flg.udp_rcvd = false;
        break;
      }
      delay(1);
    }
  }
  flg.udp_busy = false; // UDP使用中フラグをサゲる

  // @[2-2] チェックサムを確認
  if (mrd.cksm_rslt(r_udp_meridim.sval, MRDM_LEN)) // Check sum OK!
  {

    // @[2-3] UDP受信配列から UDP送信配列にデータを転写
    memcpy(s_udp_meridim.bval, r_udp_meridim.bval, MRDM_LEN * 2);

    // @[2-4a] エラービット14番(ESP32のPCからのUDP受信エラー検出)をサゲる
    mrd_clearBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_PC_ESP);

  } else { // チェックサムがNGならバッファから転記せず前回のデータを使用する

    // @[2-4b] エラービット14番(ESP32のPCからのUDP受信エラー検出)をアゲる
    mrd_setBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_PC_ESP);
    err.pc_esp++;
  }

  // @[2-5] シーケンス番号チェック
  mrdsq.r_expect = mrdsq.r_expect % 60000; // シーケンス番号予想値の生成

  if (mrd.seq_compare_nums(mrdsq.r_expect, int(s_udp_meridim.usval[MRD_SEQ]))) {

    // エラービット10番[ESP受信のスキップ検出]をサゲる
    mrd_clearBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_UDP_ESP_SKIP);
    flg.meridim_rcvd = true; // Meridim受信成功フラグをアゲる.

  } else {                                              // 受信シーケンス番号の値が予想と違ったら
    mrdsq.r_expect = int(s_udp_meridim.usval[MRD_SEQ]); // 現在の受信値を予想結果としてキープ

    // エラービット10番[ESP受信のスキップ検出]をアゲる
    mrd_setBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_UDP_ESP_SKIP);

    err.esp_skip++;
    flg.meridim_rcvd = false; // Meridim受信成功フラグをサゲる.
  }

  if (flg.meridim_rcvd) {
    // @[3-1] MasterCommand group1 の処理
    execute_master_command_1(s_udp_meridim);
    // @[6-1] MasterCommand group2 の処理
    execute_master_command_2(s_udp_meridim);
  }

  // @[7-1] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
  for (int i = 0; i <= sv.num_max; i++) {
    sv.ixl_tgt_past[i] = sv.ixl_tgt[i];                    // 前回のdegreeをキープ
    sv.ixr_tgt_past[i] = sv.ixr_tgt[i];                    // 前回のdegreeをキープ
    sv.ixl_tgt[i] = s_udp_meridim.sval[i * 2 + 21] * 0.01; // 受信したdegreeを格納
    sv.ixr_tgt[i] = s_udp_meridim.sval[i * 2 + 51] * 0.01; // 受信したdegreeを格納
  }

  // @[7-2] ESP32による次回動作の計算
  // 以下はリモコンの左十字キー左右でL系統0番サーボ(首部)を30度左右にふるサンプル
  if (s_udp_meridim.sval[MRD_PAD_BUTTONS] == PAD_RIGHT) {
    sv.ixl_tgt[0] = -30.00; // -30度
  } else if (s_udp_meridim.sval[MRD_PAD_BUTTONS] == PAD_LEFT) {
    sv.ixl_tgt[0] = 30.00; // +30度
  }

  // @[7-3] 各種処理

  // @[8-1] サーボ受信値の処理
  if (!MODE_ESP32_STDALONE) {          // サーボ処理を行うかどうか
    _servo_l->mrd_servos_drive_lite(); // サーボ動作を実行する
    _servo_r->mrd_servos_drive_lite(); // サーボ動作を実行する
  } else {
    // ボード単体動作モードの場合はサーボ処理をせずL0番サーボ値として+-30度のサインカーブ値を返す
    sv.ixl_tgt[0] = sin(tmr.count_loop * M_PI / 180.0) * 30;
  }

  // @[9-1] サーボIDごとにの現在位置もしくは計算結果を配列に格納
  for (int i = 0; i <= sv.num_max; i++) {
    // 最新のサーボ角度をdegreeで格納
    s_udp_meridim.sval[i * 2 + 21] = mrd.float2HfShort(sv.ixl_tgt[i]);
    s_udp_meridim.sval[i * 2 + 51] = mrd.float2HfShort(sv.ixr_tgt[i]);
  }
}

//================================================================================================================
//  コマンド処理
//================================================================================================================

#endif // __MERIDIAN_LITE_MAIN__
