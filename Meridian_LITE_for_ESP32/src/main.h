#ifndef __MERIDIAN_MAIN_FUNC__
#define __MERIDIAN_MAIN_FUNC__

// ヘッダファイルの読み込み
#include "mrd_common.h"

// ライブラリ導入
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用
#include <IcsHardSerialClass.h>         // ICSサーボのインスタンス設定
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用
#include <Meridian.h>                   // Meridianのライブラリ導入

//------------------------------------------------------------------------------------
//  列挙型
//------------------------------------------------------------------------------------

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

enum BinHexDec { // 数値表示タイプの列挙型(Bin, Hex, Dec)
  Bin = 0,       // BIN
  Hex = 1,       // HEX
  Dec = 2,       // DEC
};

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

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

// 6軸or9軸センサーの値
struct AhrsValue {
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス

  MPU6050 mpu6050;        // MPU6050のインスタンス
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
  float yaw_origin = 0;   // ヨー軸の補正センター値
  float yaw_source = 0;   // ヨー軸のソースデータ保持用

  float read[16]; // mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp

  float zeros[16] = {0};               // リセット用
  float ave_data[16];                  // 上記の移動平均値を入れる
  float result[16];                    // 加工後の最新のmpuデータ(二次データ)
  float stock_data[IMUAHRS_STOCK][16]; // 上記の移動平均値計算用のデータストック
  int stock_count = 0;                 // 上記の移動平均値計算用のデータストックを輪番させる時の変数
  VectorInt16 aa;                      // [x, y, z]            加速度センサの測定値
  VectorInt16 gyro;                    // [x, y, z]            角速度センサの測定値
  VectorInt16 mag;                     // [x, y, z]            磁力センサの測定値
  long temperature;                    // センサの温度測定値
};

// モニタリング設定
struct MrdMonitor {
  bool flow = MONITOR_FLOW;           // フローを表示
  bool all_err = MONITOR_ERR_ALL;     // 全経路の受信エラー率を表示
  bool servo_err = MONITOR_ERR_SERVO; // サーボエラーを表示
  bool seq_num = MONITOR_SEQ;         // シーケンス番号チェックを表示
  bool pad = MONITOR_PAD;             // リモコンのデータを表示
};

#endif //__MERIDIAN_MAIN_FUNC__
