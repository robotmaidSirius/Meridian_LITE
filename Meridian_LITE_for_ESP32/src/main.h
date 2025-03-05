#ifndef __MERIDIAN_MAIN_FUNC__
#define __MERIDIAN_MAIN_FUNC__

// ヘッダファイルの読み込み
#include "config.h"

// ライブラリ導入
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用
#include <Meridian.h>                   // Meridianのライブラリ導入
extern MERIDIANFLOW::Meridian mrd;
#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定
extern IcsHardSerialClass ics_L;
extern IcsHardSerialClass ics_R;
#include "mrd_module/sv_common.hpp" // サーボ用定義

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

enum ImuAhrsType { // 6軸9軸センサ種の列挙型(NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS)
  NO_IMU = 0,      // IMU/AHRS なし.
  MPU6050_IMU = 1, // MPU6050
  MPU9250_IMU = 2, // MPU9250(未設定)
  BNO055_AHRS = 3  // BNO055
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
//  変数
//------------------------------------------------------------------------------------

// システム用の変数
TaskHandle_t thp[4]; // マルチスレッドのタスクハンドル格納用

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

// フラグ用変数
struct MrdFlags {
  bool udp_board_passive = false; // UDP通信の周期制御がボード主導(false) か, PC主導(true)か.
  bool count_frame_reset = false; // フレーム管理時計をリセットする
  bool stop_board_during = false; // ボードの末端処理をmeridim[2]秒, meridim[3]ミリ秒だけ止める.
  bool sdcard_write_mode = false; // SDCARDへの書き込みモード.
  bool sdcard_read_mode = false;  // SDCARDからの読み込みモード.
  bool wire0_init = false;        // I2C 0系統の初期化合否
  bool wire1_init = false;        // I2C 1系統の初期化合否
  bool bt_busy = false;           // Bluetoothの受信中フラグ（UDPコンフリクト回避用）
  bool spi_rcvd = true;           // SPIのデータ受信判定
  bool udp_rcvd = false;          // UDPのデータ受信判定
  bool udp_busy = false;          // UDPスレッドでの受信中フラグ（送信抑制）

  bool udp_receive_mode = MODE_UDP_RECEIVE; // PCからのデータ受信実施（0:OFF, 1:ON, 通常は1）
  bool udp_send_mode = MODE_UDP_SEND;       // PCへのデータ送信実施（0:OFF, 1:ON, 通常は1）
  bool meridim_rcvd = false;                // Meridimが正しく受信できたか.
};

// シーケンス番号理用の変数
struct MrdSq {
  int s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
  int r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};

// タイマー管理用の変数
struct MrdTimer {
  long frame_ms = FRAME_DURATION; // 1フレームあたりの単位時間(ms)
  int count_loop = 0;             // サイン計算用の循環カウンタ
  int count_loop_dlt = 2;         // サイン計算用の循環カウンタを1フレームにいくつ進めるか
  int count_loop_max = 359999;    // 循環カウンタの最大値
  unsigned long count_frame = 0;  // メインフレームのカウント

  int pad_interval = (PAD_INTERVAL - 1 > 0) ? PAD_INTERVAL - 1 : 1; // パッドの問い合わせ待機時間
};

// エラーカウント用
struct MrdErr {
  int esp_pc = 0;   // PCの受信エラー（ESP32からのUDP）
  int pc_esp = 0;   // ESP32の受信エラー（PCからのUDP）
  int esp_tsy = 0;  // Teensyの受信エラー（ESP32からのSPI）
  int tsy_esp = 0;  // ESP32の受信エラー（TeensyからのSPI）
  int esp_skip = 0; // UDP→ESP受信のカウントの連番スキップ回数
  int tsy_skip = 0; // ESP→Teensy受信のカウントの連番スキップ回数
  int pc_skip = 0;  // PC受信のカウントの連番スキップ回数
};

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

// モニタリング設定
struct MrdMonitor {
  bool flow = MONITOR_FLOW;           // フローを表示
  bool all_err = MONITOR_ERR_ALL;     // 全経路の受信エラー率を表示
  bool servo_err = MONITOR_ERR_SERVO; // サーボエラーを表示
  bool seq_num = MONITOR_SEQ;         // シーケンス番号チェックを表示
  bool pad = MONITOR_PAD;             // リモコンのデータを表示
};

//==================================================================================================
//  関数各種
//==================================================================================================

///@brief Generate expected sequence number from input.
///@param a_previous_num Previous sequence number.
///@return Expected sequence number. (0 to 59,999)
uint16_t mrd_seq_predict_num(uint16_t a_previous_num) {
  uint16_t x_tmp = a_previous_num + 1;
  if (x_tmp > 59999) // Reset counter
  {
    x_tmp = 0;
  }
  return x_tmp;
}

#endif //__MERIDIAN_MAIN_FUNC__
