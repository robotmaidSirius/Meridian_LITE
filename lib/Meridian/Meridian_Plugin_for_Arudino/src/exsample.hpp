

#define VERSION "Meridian_LITE_v1.1.1_2024_08.18" // バージョン表示

/// @file    Meridian_LITE_for_ESP32/src/main.cpp
/// @brief   Meridian is a system that smartly realizes the digital twin of a robot.
/// @details Meridian_LITE for Meridian Board -LITE- with ESP32DecKitC.
///
/// This code is licensed under the MIT License.
/// Copyright (c) 2022 Izumi Ninagawa & Project Meridian

//================================================================================================================
//  初期設定
//================================================================================================================

//================================================================================================================
//  サーボIDとロボット部位、軸との対応表 (KHR-3HVの例)
//================================================================================================================
//
// ID    Parts/Axis ＜ICS_Left_Upper SIO1,SIO2＞
// [L00] 頭/ヨー
// [L01] 左肩/ピッチ
// [L02] 左肩/ロール
// [L03] 左肘/ヨー
// [L04] 左肘/ピッチ
// [L05] -
// ID    Parts/Axis ＜ICS_Left_Lower SIO3,SIO4＞
// [L06] 左股/ロール
// [L07] 左股/ピッチ
// [L08] 左膝/ピッチ
// [L09] 左足首/ピッチ
// [L10] 左足首/ロール
// ID    Parts/Axis  ＜ICS_Right_Upper SIO5,SIO6＞
// [R00] 腰/ヨー
// [R01] 右肩/ピッチ
// [R02] 右肩/ロール
// [R03] 右肘/ヨー
// [R04] 右肘/ピッチ
// [R05] -
// ID    Parts/Axis  ＜ICS_Right_Lower SIO7,SIO8＞
// [R06] 右股/ロール
// [R07] 右股/ピッチ
// [R08] 右膝/ピッチ
// [R09] 右足首/ピッチ
// [R10] 右足首/ロール
// ヘッダファイルの読み込み
#include "config.h"

// ライブラリ導入
#include "Board/meridian_board_lite_for_esp32.hpp"

// ヘッダファイルの読み込み

#include "mrd_app/motion/mrd_move.h"
#include "mrd_module/filesystem/mrd_eeprom.h"
#include "mrd_module/filesystem/mrd_sd.h"
#include "mrd_module/network/mrd_wifi_esp32.hpp"
#include "mrd_module/servo/mrd_servo_kondo_ics_3_5.hpp"

#if defined(AHRS_TYPE_MPU9250) // MPU9250の場合
#include "mrd_module/ahrs/mrd_imu_MPU9250.hpp"
I_Meridian_AHRS *plugin_ahrs = new MrdImuMPU9250();
#elif defined(AHRS_TYPE_BNO055) // BNO055の場合
#include "mrd_module/ahrs/mrd_ahrs_BNO055.hpp"
I_Meridian_AHRS *plugin_ahrs = new MrdAhrsBNO055();
#elif defined(AHRS_TYPE_MPU6050) // BNO055の場合
#include "mrd_module/ahrs/mrd_imu_MPU6050.hpp"
I_Meridian_AHRS *plugin_ahrs = new MrdImuMPU6050();
#else
#include "mrd_plugin/i_mrd_ahrs.hpp"
I_Meridian_AHRS *plugin_ahrs = NULL;
#endif

#if defined(PAD_TYPE_WIIMOTE) // Wiiリモコンの場合
#include "mrd_module/joypad/mrd_joypad_wiimote.hpp"
MrdPadWiimote *plugin_joypad = new MrdPadWiimote();
#elif defined(PAD_TYPE_KRC5FH) // KRC-5FHの場合
#include "mrd_module/joypad/mrd_joypad_krc5fh.hpp"
MrdJoypadKrc5fh *plugin_joypad = new MrdJoypadKrc5fh();
#else
#include <mrd_plugin/i_mrd_joypad.hpp>
I_Meridian_Joypad *plugin_joypad = NULL;
#endif

MrdServoKondoIcs35 *plugin_servo = new MrdServoKondoIcs35(Serial1, Serial2);
MrdWifiESP32 *mrd_wifi = new MrdWifiESP32();

//------------------------------------------------------------------------------------
//  変数
//------------------------------------------------------------------------------------

// システム用の変数
const int MRD_ERR = MRDM_LEN - 2;      // エラーフラグの格納場所（配列の末尾から2つめ）
const int MRD_ERR_u = MRD_ERR * 2 + 1; // エラーフラグの格納場所（上位8ビット）
const int MRD_ERR_l = MRD_ERR * 2;     // エラーフラグの格納場所（下位8ビット）
const int MRD_CKSM = MRDM_LEN - 1;     // チェックサムの格納場所（配列の末尾）
TaskHandle_t thp[4];                   // マルチスレッドのタスクハンドル格納用

// ハードウェアタイマーとカウンタ用変数の定義
hw_timer_t *timer = NULL;                              // ハードウェアタイマーの設定
volatile SemaphoreHandle_t timer_semaphore;            // ハードウェアタイマー用のセマフォ
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED; // ハードウェアタイマー用のミューテックス
unsigned long count_frame = 0;                         // フレーム処理の完了時にカウントアップ
volatile unsigned long count_timer = 0;                // フレーム用タイマーのカウントアップ

// Meridim配列用の共用体の設定
Meridim90Union s_udp_meridim;       // Meridim配列データ送信用(short型, センサや角度は100倍値)
Meridim90Union r_udp_meridim;       // Meridim配列データ受信用
Meridim90Union s_udp_meridim_dummy; // SPI送信ダミー用

ServoParam sv;
// シーケンス番号理用の変数
struct MrdSq {
  int s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
  int r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};
MrdSq mrdsq;
// タイマー管理用の変数
struct MrdTimer {
  long frame_ms = FRAME_DURATION; // 1フレームあたりの単位時間(ms)
  int count_loop = 0;             // サイン計算用の循環カウンタ
  int count_loop_dlt = 2;         // サイン計算用の循環カウンタを1フレームにいくつ進めるか
  int count_loop_max = 359999;    // 循環カウンタの最大値
  unsigned long count_frame = 0;  // メインフレームのカウント
};
MrdTimer tmr;
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
MrdErr err;
// モニタリング設定
struct MrdMonitor {
  bool flow = MONITOR_FLOW;           // フローを表示
  bool all_err = MONITOR_ERR_ALL;     // 全経路の受信エラー率を表示
  bool servo_err = MONITOR_ERR_SERVO; // サーボエラーを表示
  bool seq_num = MONITOR_SEQ;         // シーケンス番号チェックを表示
  bool pad = MONITOR_PAD;             // リモコンのデータを表示
};
MrdMonitor monitor;

// フラグ用変数
struct MrdFlags {
  bool imuahrs_available = true;            // メインセンサ値を読み取る間, サブスレッドによる書き込みを待機
  bool udp_board_passive = false;           // UDP通信の周期制御がボード主導(false) か, PC主導(true)か.
  bool count_frame_reset = false;           // フレーム管理時計をリセットする
  bool stop_board_during = false;           // ボードの末端処理をmeridim[2]秒, meridim[3]ミリ秒だけ止める.
  bool eeprom_write_mode = false;           // EEPROMへの書き込みモード.
  bool eeprom_read_mode = false;            // EEPROMからの読み込みモード.
  bool eeprom_protect = EEPROM_PROTECT;     // EEPROMの書き込みプロテクト.
  bool eeprom_load = EEPROM_LOAD;           // 起動時にEEPROMの内容を読み込む
  bool eeprom_set = EEPROM_SET;             // 起動時にEEPROMに規定値をセット
  bool sdcard_write_mode = false;           // SDCARDへの書き込みモード.
  bool sdcard_read_mode = false;            // SDCARDからの読み込みモード.
  bool wire0_init = false;                  // I2C 0系統の初期化合否
  bool wire1_init = false;                  // I2C 1系統の初期化合否
  bool bt_busy = false;                     // Bluetoothの受信中フラグ（UDPコンフリクト回避用）
  bool spi_rcvd = true;                     // SPIのデータ受信判定
  bool udp_rcvd = false;                    // UDPのデータ受信判定
  bool udp_busy = false;                    // UDPスレッドでの受信中フラグ（送信抑制）
  bool udp_receive_mode = MODE_UDP_RECEIVE; // PCからのデータ受信実施（0:OFF, 1:ON, 通常は1）
  bool udp_send_mode = MODE_UDP_SEND;       // PCへのデータ送信実施（0:OFF, 1:ON, 通常は1）
  bool meridim_rcvd = false;                // Meridimが正しく受信できたか.
};
MrdFlags flg;

/// @brief count_timerを保護しつつ1ずつインクリメント
void IRAM_ATTR frame_timer() {
  portENTER_CRITICAL_ISR(&timer_mux);
  count_timer++;
  portEXIT_CRITICAL_ISR(&timer_mux);
  xSemaphoreGiveFromISR(timer_semaphore, NULL); // セマフォを与える
}
///@brief Generate expected sequence number from input.
///@param a_previous_num Previous sequence number.
///@return Expected sequence number. (0 to 59,999)
inline uint16_t mrd_seq_predict_num(uint16_t a_previous_num) {
#if 1
  return (a_previous_num + 1) % 60000; // Reset counter
#else
  uint16_t x_tmp = a_previous_num + 1;
  if (x_tmp > 59999) // Reset counter
  {
    x_tmp = 0;
  }
  return x_tmp;
#endif
}

// 予約用
bool execute_master_command_1(Meridim90Union a_meridim, bool a_flg_exe);
bool execute_master_command_2(Meridim90Union a_meridim, bool a_flg_exe);
//================================================================================================================
//  シリアルモニタ表示用の関数
//================================================================================================================

class MrdMsgHandler {
private:
  Stream &m_serial; // シリアルオブジェクトの参照を保持

public:
  enum UartLine { // サーボ系統の列挙型(L,R,C)
    L,            // Left
    R,            // Right
    C             // Center
  };
  // コンストラクタでStreamオブジェクトを受け取り, メンバーに保存
  MrdMsgHandler(Stream &a_serial) : m_serial(a_serial) {}

  //------------------------------------------------------------------------------------
  //  起動時メッセージ
  //------------------------------------------------------------------------------------

  /// @brief 指定されたミリ秒数だけキャパシタの充電プロセスを示すメッセージを表示する.
  /// @param a_mill 充電プロセスの期間を秒単位で指定.
  void charging(int a_mill) {
    m_serial.print("Charging the capacitor.");
    for (int i = 0; i < a_mill; i++) {
      if (i % 100 == 0) { // 100msごとにピリオドを表示
        m_serial.print(".");
      }
      delay(1);
    }
    m_serial.println();
  }

  /// @brief システムのバージョン情報と通信速度の設定を表示するためのメッセージを出力する.
  /// @param a_version バージョン情報.
  /// @param a_pc_speed PCとのUSBシリアル通信速度.
  /// @param a_spi_speed SPIの通信速度.
  /// @param a_i2c0_speed I2C0の通信速度.
  void hello_lite_esp(String a_version, int a_pc_speed, int a_spi_speed, int a_i2c0_speed) {
    m_serial.println();
    m_serial.print("Hi, This is ");
    m_serial.println(a_version);
    m_serial.print("Set PC-USB ");
    m_serial.print(a_pc_speed);
    m_serial.println(" bps");
    m_serial.print("Set SPI0   ");
    m_serial.print(a_spi_speed);
    m_serial.println(" bps");
    m_serial.print("Set i2c0   ");
    m_serial.print(a_i2c0_speed);
    m_serial.println(" bps");
  }

  /// @brief マウント設定したサーボのbpsをシリアルモニタに出力する.
  /// @param a_servo_l L系統のサーボbps.
  /// @param a_servo_r R系統のサーボbps.
  void servo_bps_2lines(int a_servo_l, int a_servo_r) {
    m_serial.print("Set UART_L ");
    m_serial.print(a_servo_l);
    m_serial.println(" bps");
    m_serial.print("Set UART_R ");
    m_serial.print(a_servo_r);
    m_serial.println(" bps");
  }

  /// @brief 指定されたUARTラインのサーボIDを表示する.
  /// @param a_label UARTラインのラベル.
  /// @param a_max サーボの最大数.
  /// @param a_mount サーボのマウント状態を示す配列.
  /// @param a_id サーボIDの配列.
  void print_servo_ids(const char *a_label, int a_max, int *a_mount, const int *a_id) {
    m_serial.print(a_label);
    for (int i = 0; i <= a_max; i++) {
      if (a_mount[i] != 0) {
        if (a_id[i] < 10) {
          m_serial.print(" ");
        }
        m_serial.print(a_id[i]);
      } else {
        m_serial.print("__");
      }
      m_serial.print(" ");
    }
    m_serial.println();
  }

  /// @brief マウントされているサーボのIDを表示する.
  /// @param a_sv サーボパラメータの構造体.
  void servo_mounts_2lines(ServoParam a_sv) {
    print_servo_ids("UART_L Servos mounted: ", a_sv.num_max, a_sv.ixl_mount, a_sv.ixl_id);
    print_servo_ids("UART_R Servos mounted: ", a_sv.num_max, a_sv.ixr_mount, a_sv.ixr_id);
  }

  /// @brief wifiの接続開始メッセージを出力する.
  /// @param a_ssid 接続先のSSID.
  void esp_wifi(const char *a_ssid) {
    m_serial.println("WiFi connecting to => " + String(a_ssid)); // WiFi接続完了通知
  }

  /// @brief 指定されたUARTラインとサーボタイプに基づいてサーボの通信プロトコルを表示する.
  /// @param a_line UART通信ライン（L, R, またはC）.
  /// @param a_servo_type サーボのタイプを示す整数値.
  /// @return サーボがサポートされている場合はtrueを, サポートされていない場合はfalseを返す.
  bool servo_protocol(UartLine a_line, int a_servo_type) {
    if (a_servo_type > 0) {
      m_serial.print("Set UART_");
      m_serial.print(this->mrd_get_line_name(a_line));
      m_serial.print(" protocol : ");

      switch (a_servo_type) {
      case 1:
        m_serial.print("single PWM");
        m_serial.println(" - Not supported yet.");
        break;
      case 11:
        m_serial.print("I2C_PCA9685 to PWM");
        m_serial.println(" - Not supported yet.");
        break;
      case 21:
        m_serial.print("RSxTTL (FUTABA)");
        m_serial.println(" - Not supported yet.");
        break;
      case 31:
        m_serial.print("DYNAMIXEL Protocol 1.0");
        m_serial.println(" - Not supported yet.");
        break;
      case 32:
        m_serial.print("DYNAMIXEL Protocol 2.0");
        m_serial.println(" - Not supported yet.");
        break;
      case 43:
        m_serial.println("ICS3.5/3.6(KONDO,KRS)");
        break;
      case 44:
        m_serial.print("PMX(KONDO)");
        m_serial.println(" - Not supported yet.");
        break;
      case 51:
        m_serial.print("XBUS(JR PROPO)");
        m_serial.println(" - Not supported yet.");
        break;
      case 61:
        m_serial.print("STS(FEETECH)");
        m_serial.println(" - Not supported yet.");
        break;
      case 62:
        m_serial.print("SCS(FEETECH)");
        m_serial.println(" - Not supported yet.");
        break;
      default:
        m_serial.println(" Not defined. ");
        break;
      }
      return true;
    }
    return false;
  }

  /// @brief wifiの接続完了メッセージと各IPアドレスを出力する.
  /// @param a_flg_fixed_ip 固定IPかどうか. true:固定IP, false:動的IP.
  /// @param a_ssid 接続先のSSID.
  /// @param a_fixedip 固定IPの場合の値.
  void esp_ip(bool a_flg_fixed_ip, const char *a_ssid, const char *a_fixedip) {
    m_serial.println("WiFi successfully connected.");                      // WiFi接続完了通知
    m_serial.println("PC's IP address target => " + String(WIFI_SEND_IP)); // 送信先PCのIPアドレスの表示

    if (a_flg_fixed_ip) {
      m_serial.println("ESP32's IP address => " + String(FIXED_IP_ADDR) + " (*Fixed)"); // ESP32自身のIPアドレスの表示
    } else {
      m_serial.print("ESP32's IP address => "); // ESP32自身のIPアドレスの表示
      m_serial.println(WiFi.localIP().toString());
    }
  }

  /// @brief マウント設定したジョイパッドのタイプをシリアルモニタに出力する.
  /// @param a_mount_pad パッドの定義(PC,MERIMOTE,BLUERETRO,SBDBT,KRR5FH,WIIMOTE)
  void mounted_pad() {
    if (NULL != plugin_joypad) {
      m_serial.printf("Pad Receiver mounted : %d", plugin_joypad->get_name());
    } else {
      m_serial.println("Pad Receiver mounted : None");
    }
  }

  /// @brief システムの動作開始を示すメッセージを出力する.
  void flow_start_lite_esp() {
    m_serial.println();
    m_serial.println("-) Meridian -LITE- system on ESP32 now flows. (-");
  }

  //------------------------------------------------------------------------------------
  //  イベントメッセージ
  //------------------------------------------------------------------------------------

  /// @brief システム内の様々な通信エラーとスキップ数をモニタリングし, シリアルポートに出力する.
  /// @param mrd_disp_all_err モニタリング表示のオンオフ.
  /// @param a_err エラーデータの入った構造体.
  /// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
  bool all_err(bool mrd_disp_all_err, MrdErr a_err) {
    if (mrd_disp_all_err) {
      m_serial.print("[ERR] es>pc:");
      m_serial.print(a_err.esp_pc);
      m_serial.print(" pc>es:");
      m_serial.print(a_err.pc_esp);
      m_serial.print(" es>ts:");
      m_serial.print(a_err.esp_tsy);
      m_serial.print(" ts>es:");
      m_serial.print(a_err.esp_tsy);
      m_serial.print(" tsSkp:");
      m_serial.print(a_err.tsy_skip);
      m_serial.print(" esSkp:");
      m_serial.print(a_err.esp_skip);
      m_serial.print(" pcSkp:");
      m_serial.print(a_err.pc_skip);
      m_serial.println();
      return true;
    }
    return false;
  }

  /// @brief サーボモーターのエラーを検出した場合にエラーメッセージを表示する.
  /// @param a_line サーボモーターが接続されているUARTライン（L, R, C）.
  /// @param a_num エラーが発生しているサーボの番号.
  /// @param a_flg_disp エラーメッセージを表示するかどうかのブール値.
  /// @return エラーメッセージが表示された場合はtrueを, 表示されなかった場合はfalseを返す.
  bool servo_err(UartLine a_line, int a_num, bool a_flg_disp) {
    if (a_flg_disp) {
      m_serial.print("Found servo err ");
      if (a_line == UartLine::L) {
        m_serial.print("L_");
        m_serial.println(a_num);
        return true;
      } else if (a_line == UartLine::R) {
        m_serial.print("R_");
        m_serial.println(a_num);
        return true;
      } else if (a_line == UartLine::C) {
        m_serial.print("C_");
        m_serial.println(a_num);
        return true;
      }
    }
    return false;
  }

  /// @brief 期待するシーケンス番号と実施に受信したシーケンス番号を表示する.
  /// @param a_seq_expect 期待するシーケンス番号.
  /// @param a_seq_rcvd 実際に受信したシーケンス番号.
  /// @param a_disp_seq_num 表示するかどうかのブール値.
  /// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
  bool seq_number(uint16_t a_seq_expect, uint16_t a_seq_rcvd, bool a_disp) {
    if (a_disp) {
      m_serial.print("Seq ep/rv ");
      m_serial.print(a_seq_expect);
      m_serial.print("/");
      m_serial.println(a_seq_rcvd);
      return true;
    }
    return false;
  }
  /// @brief 列挙型(L,R,C)から文字列を取得する関数.
  /// @param a_line 列挙型 enum UartLine
  /// @return 列挙型の内容に応じて文字列"L","R","C"返す.
  const char *mrd_get_line_name(UartLine a_line) {
    switch (a_line) {
    case UartLine::L:
      return "L";
    case UartLine::R:
      return "R";
    case UartLine::C:
      return "C";
    default:
      return "Unknown";
    }
  }
};
MrdMsgHandler mrd_disp(Serial);

//================================================================================================================
//  SETUP
//================================================================================================================
void setup() {

  // BT接続確認用LED設定
  pinMode(PIN_LED_BT, OUTPUT);
  digitalWrite(PIN_LED_BT, HIGH);

  // シリアルモニターの設定
  Serial.begin(SERIAL_PC_BPS);
  // シリアルモニターの確立待ち
  unsigned long start_time = millis();
  while (!Serial && (millis() - start_time < SERIAL_PC_TIMEOUT)) { // タイムアウトもチェック
    delay(1);
  }

  // ピンモードの設定
  pinMode(PIN_ERR_LED, OUTPUT); // エラー通知用LED

  // ボード搭載のコンデンサの充電時間として待機
  mrd_disp.charging(CHARGE_TIME);

  // 起動メッセージの表示(バージョン, PC-USB,SPI0,i2c0のスピード)
  mrd_disp.hello_lite_esp(VERSION, SERIAL_PC_BPS, SPI0_SPEED, I2C0_SPEED);

  // サーボ値の初期設定
  sv.num_max = max(mrd_max_used_index(IXL_MT, IXL_MAX),  //
                   mrd_max_used_index(IXR_MT, IXR_MAX)); // サーボ処理回数
  for (int i = 0; i <= sv.num_max; i++) {                // configで設定した値を反映させる
    sv.ixl_mount[i] = IXL_MT[i];
    sv.ixr_mount[i] = IXR_MT[i];
    sv.ixl_id[i] = IXL_ID[i];
    sv.ixr_id[i] = IXR_ID[i];
    sv.ixl_cw[i] = IXL_CW[i];
    sv.ixr_cw[i] = IXR_CW[i];
    sv.ixl_trim[i] = IDL_TRIM[i];
    sv.ixr_trim[i] = IDR_TRIM[i];
  };

// サーボ用UART設定
#if 1
  if (true == plugin_servo->begin()) { // サーボモータの通信初期設定.
    // サーボUARTの通信速度の表示
    mrd_disp.servo_bps_2lines(SERVO_BAUDRATE_L, SERVO_BAUDRATE_R);
  }
#else
  mrd_servo_begin(L, MOUNT_SERVO_TYPE_L); // サーボモータの通信初期設定. Serial2
  mrd_servo_begin(R, MOUNT_SERVO_TYPE_R); // サーボモータの通信初期設定. Serial3
#endif
  mrd_disp.servo_protocol(MrdMsgHandler::L, MOUNT_SERVO_TYPE_R); // サーボプロトコルの表示
  mrd_disp.servo_protocol(MrdMsgHandler::R, MOUNT_SERVO_TYPE_R);

  // マウントされたサーボIDの表示
  mrd_disp.servo_mounts_2lines(sv);

  // EEPROMの開始, ダンプ表示
  mrd_eeprom_init(EEPROM_SIZE);                                     // EEPROMの初期化
  mrd_eeprom_dump_at_boot(EEPROM_DUMP);                             // 内容のダンプ表示
  mrd_eeprom_write_read_check(mrd_eeprom_make_data_from_config(sv), //
                              CHECK_EEPROM_RW,                      //
                              EEPROM_PROTECT,                       //
                              flg.eeprom_protect);                  // EEPROMのリードライトテスト

  // SDカードの初期設定とチェック
  mrd_sd_init(PIN_CHIPSELECT_SD);
  mrd_sd_check();

  // I2Cの初期化と開始
  if (NULL != plugin_ahrs) {
    if (true == plugin_ahrs->setup()) {
      // mrd_wire0_setup(BNO055_AHRS, I2C0_SPEED, ahrs, PIN_I2C0_SDA, PIN_I2C0_SCL);

      // I2Cスレッドの開始
      if (true == plugin_ahrs->begin()) {
        // if (true == mrd_ahrs_setup(thp[0])) {
        Serial.println("Core0 thread for BNO055 start.");
        delay(10);
      }
    }
  }

  // WiFiの初期化と開始
  if (NULL != mrd_wifi) {
    mrd_disp.esp_wifi(WIFI_AP_SSID);
    if (mrd_wifi->mrd_wifi_init(WIFI_AP_SSID, WIFI_AP_PASS, Serial)) {
      // wifiIPの表示
      mrd_disp.esp_ip(NETWORK_MODE_FIXED_IP, WIFI_SEND_IP, FIXED_IP_ADDR);
    }
  }

  // コントロールパッドの種類を表示
  if (NULL != plugin_joypad) {
    if (true == plugin_joypad->mrd_joypad_setup(thp[2], Serial)) {
      mrd_disp.mounted_pad();
    }
  }

  // UDP開始用ダミーデータの生成
  s_udp_meridim.sval[MRD_MASTER] = 90;
  s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
  r_udp_meridim.sval[MRD_MASTER] = 90;
  r_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(r_udp_meridim.sval, MRDM_LEN);

  // タイマーの設定
  timer_semaphore = xSemaphoreCreateBinary();          // セマフォの作成
  timer = timerBegin(0, 80, true);                     // タイマーの設定（1つ目のタイマーを使用, 分周比80）
  timerAttachInterrupt(timer, &frame_timer, true);     // frame_timer関数をタイマーの割り込みに登録
  timerAlarmWrite(timer, FRAME_DURATION * 1000, true); // タイマーを10msごとにトリガー
  timerAlarmEnable(timer);                             // タイマーを開始

  // 開始メッセージ
  mrd_disp.flow_start_lite_esp();

  // タイマーの初期化
  count_frame = 0;
  portENTER_CRITICAL(&timer_mux);
  count_timer = 0;
  portEXIT_CRITICAL(&timer_mux);
}

//================================================================================================================
// MAIN LOOP
//================================================================================================================
void loop() {
  //------------------------------------------------------------------------------------
  //  [ 1 ] UDP送信
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[1]", monitor.flow); // デバグ用フロー表示

  // @[1-1] UDP送信の実行
  if (flg.udp_send_mode) // UDPの送信実施フラグの確認（モード確認）
  {
    flg.udp_busy = true; // UDP使用中フラグをアゲる
    if (NULL != mrd_wifi) {
      mrd_wifi->mrd_wifi_udp_send(s_udp_meridim.bval, MRDM_BYTE);
    }
    flg.udp_busy = false; // UDP使用中フラグをサゲる
    flg.udp_rcvd = false; // UDP受信完了フラグをサゲる
  }

  //------------------------------------------------------------------------------------
  //  [ 2 ] UDP受信
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[2]", monitor.flow); // デバグ用フロー表示

  // @[2-1] UDPの受信待ち受けループ
  if (flg.udp_receive_mode) // UDPの受信実施フラグの確認（モード確認）
  {
    unsigned long start_tmp = millis();
    flg.udp_busy = true;  // UDP使用中フラグをアゲる
    flg.udp_rcvd = false; // UDP受信完了フラグをサゲる
    while (!flg.udp_rcvd) {
      // UDP受信処理
      if (NULL != mrd_wifi) {
        if (mrd_wifi->mrd_wifi_udp_receive(r_udp_meridim.bval, MRDM_BYTE)) // 受信確認
        {
          flg.udp_rcvd = true; // UDP受信完了フラグをアゲる
        }
      } else {
        flg.udp_rcvd = true; // UDP受信完了フラグをアゲる
      }

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
    mrd.monitor_check_flow("CsOK", monitor.flow); // デバグ用フロー表示

    // @[2-3] UDP受信配列から UDP送信配列にデータを転写
    memcpy(s_udp_meridim.bval, r_udp_meridim.bval, MRDM_LEN * 2);

    // @[2-4a] エラービット14番(ESP32のPCからのUDP受信エラー検出)をサゲる
    mrd_clearBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);

  } else // チェックサムがNGならバッファから転記せず前回のデータを使用する
  {

    // @[2-4b] エラービット14番(ESP32のPCからのUDP受信エラー検出)をアゲる
    mrd_setBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);
    err.pc_esp++;
    mrd.monitor_check_flow("CsErr*", monitor.flow); // デバグ用フロー表示
  }

  // @[2-5] シーケンス番号チェック
  mrdsq.r_expect = mrd_seq_predict_num(mrdsq.r_expect); // シーケンス番号予想値の生成

  // @[2-6] シーケンス番号のシリアルモニタ表示
  mrd_disp.seq_number(mrdsq.r_expect, r_udp_meridim.usval[MRD_SEQ], monitor.seq_num);

  if (mrd.seq_compare_nums(mrdsq.r_expect, int(s_udp_meridim.usval[MRD_SEQ]))) {

    // エラービット10番[ESP受信のスキップ検出]をサゲる
    mrd_clearBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_10_UDP_ESP_SKIP);
    flg.meridim_rcvd = true; // Meridim受信成功フラグをアゲる.

  } else {                                              // 受信シーケンス番号の値が予想と違ったら
    mrdsq.r_expect = int(s_udp_meridim.usval[MRD_SEQ]); // 現在の受信値を予想結果としてキープ

    // エラービット10番[ESP受信のスキップ検出]をアゲる
    mrd_setBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_10_UDP_ESP_SKIP);

    err.esp_skip++;
    flg.meridim_rcvd = false; // Meridim受信成功フラグをサゲる.
  }

  //------------------------------------------------------------------------------------
  //  [ 3 ] MasterCommand group1 の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[3]", monitor.flow); // デバグ用フロー表示

  // @[3-1] MasterCommand group1 の処理
  execute_master_command_1(s_udp_meridim, flg.meridim_rcvd);

  //------------------------------------------------------------------------------------
  //  [ 4 ] センサー類読み取り
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[4]", monitor.flow); // デバグ用フロー表示

  // @[4-1] センサ値のMeridimへの転記
  flg.imuahrs_available = false;
  plugin_ahrs->refresh(s_udp_meridim); // センサー値の更新
  flg.imuahrs_available = true;

  //------------------------------------------------------------------------------------
  //  [ 5 ] リモコンの読み取り
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[5]", monitor.flow); // デバグ用フロー表示

  // @[5-1] リモコンデータの書き込み
  if (NULL != plugin_joypad) {
    // リモコンがマウントされていれば
#if defined(PAD_TYPE_WIIMOTE)
    // Wiiリモコンの場合

    // リモコンデータの読み込み
    joypad->pad_array.ui64val = joypad->mrd_pad_read(joypad->pad_array.ui64val);

    // リモコンの値をmeridimに格納する
    joypad->meriput90_pad(s_udp_meridim, joypad->pad_array, PAD_BUTTON_MARGE);
#elif defined(PAD_TYPE_KRC5FH)
    // KRC-5FHの場合

    bool rcvd;
    unsigned short krr_button;
    int krr_analog[4];
    if (NULL != plugin_servo) {
      if (true == plugin_servo->ics_R->getKrrAllData(&krr_button, krr_analog)) {

        // リモコンデータの読み込み
        plugin_joypad->pad_array.ui64val = plugin_joypad->mrd_pad_read(krr_button, krr_analog);

        // リモコンの値をmeridimに格納する
        plugin_joypad->meriput90_pad(s_udp_meridim, plugin_joypad->pad_array, PAD_BUTTON_MARGE);
      }
    }
#endif
  }

  //------------------------------------------------------------------------------------
  //  [ 6 ] MasterCommand group2 の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[6]", monitor.flow); // デバグ用フロー表示

  // @[6-1] MasterCommand group2 の処理
  execute_master_command_2(s_udp_meridim, flg.meridim_rcvd);

  //------------------------------------------------------------------------------------
  //  [ 7 ] ESP32内部で位置制御する場合の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[7]", monitor.flow); // デバグ用フロー表示

  // @[7-1] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
  for (int i = 0; i <= sv.num_max; i++) {
    sv.ixl_tgt_past[i] = sv.ixl_tgt[i];                    // 前回のdegreeをキープ
    sv.ixr_tgt_past[i] = sv.ixr_tgt[i];                    // 前回のdegreeをキープ
    sv.ixl_tgt[i] = s_udp_meridim.sval[i * 2 + 21] * 0.01; // 受信したdegreeを格納
    sv.ixr_tgt[i] = s_udp_meridim.sval[i * 2 + 51] * 0.01; // 受信したdegreeを格納
  }

  // @[7-2] ESP32による次回動作の計算
  // 以下はリモコンの左十字キー左右でL系統0番サーボ（首部）を30度左右にふるサンプル
  if (s_udp_meridim.sval[MRD_PAD_BUTTONS] == 0x20) {
    sv.ixl_tgt[0] = -30.00; // -30度
  } else if (s_udp_meridim.sval[MRD_PAD_BUTTONS] == 0x80) {
    sv.ixl_tgt[0] = 30.00; // +30度
  }

  // @[7-3] 各種処理

  //------------------------------------------------------------------------------------
  //  [ 8 ] サーボ動作の実行
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[8]", monitor.flow); // デバグ用フロー表示

  // @[8-1] サーボ受信値の処理
  if (!MODE_ESP32_STANDALONE) { // サーボ処理を行うかどうか
    if (NULL != plugin_servo) {
      plugin_servo->mrd_servos_drive_lite(s_udp_meridim, sv); // サーボ動作を実行する
    }
  } else {
    // ボード単体動作モードの場合はサーボ処理をせずL0番サーボ値として+-30度のサインカーブ値を返す
    sv.ixl_tgt[0] = sin(tmr.count_loop * M_PI / 180.0) * 30;
  }

  //------------------------------------------------------------------------------------
  //  [ 9 ] サーボ受信値の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[9]", monitor.flow); // デバグ用フロー表示

  // @[9-1] サーボIDごとにの現在位置もしくは計算結果を配列に格納
  for (int i = 0; i <= sv.num_max; i++) {
    // 最新のサーボ角度をdegreeで格納
    s_udp_meridim.sval[i * 2 + 21] = mrd.float2HfShort(sv.ixl_tgt[i]);
    s_udp_meridim.sval[i * 2 + 51] = mrd.float2HfShort(sv.ixr_tgt[i]);
  }

  //------------------------------------------------------------------------------------
  //  [ 10 ] エラーリポートの作成
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[10]", monitor.flow); // デバグ用フロー表示

  // @[10-1] エラーリポートの表示
  // mrd_msg_all_err(err, monitor.all_err);
  mrd_disp.all_err(MONITOR_ERR_ALL, err);

  //------------------------------------------------------------------------------------
  //  [ 11 ] UDP送信信号作成
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[11]", monitor.flow); // デバグ用フロー表示

  // @[11-1] フレームスキップ検出用のカウントをカウントアップして送信用に格納
  mrdsq.s_increment = mrd.seq_increase_num(mrdsq.s_increment);
  s_udp_meridim.usval[1] = mrdsq.s_increment;

  // @[11-2] エラーが出たサーボのインデックス番号を格納
  if (NULL != plugin_servo) {
    s_udp_meridim.ubval[MRD_ERR_l] = plugin_servo->mrd_servos_make_errcode_lite(sv);
  }

  // @[11-3] チェックサムを計算して格納
  // s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
  mrd_checksum(s_udp_meridim);

  //------------------------------------------------------------------------------------
  //   [ 12 ] フレーム終端処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[12]", monitor.flow); // 動作チェック用シリアル表示

  // @[12-1] count_timerがcount_frameに追いつくまで待機
  count_frame++;
  while (true) {
    if (xSemaphoreTake(timer_semaphore, 0) == pdTRUE) {
      portENTER_CRITICAL(&timer_mux);
      unsigned long current_count_timer = count_timer; // ハードウェアタイマーの値を読む
      portEXIT_CRITICAL(&timer_mux);
      if (current_count_timer >= count_frame) {
        break;
      }
    }
  }

  // @[12-2] 必要に応じてフレームの遅延累積時間frameDelayをリセット
  if (flg.count_frame_reset) {
    portENTER_CRITICAL(&timer_mux);
    count_frame = count_timer;
    portEXIT_CRITICAL(&timer_mux);
  }

  mrd.monitor_check_flow("\n", monitor.flow); // 動作チェック用シリアル表示
}

//================================================================================================================
//  コマンド処理
//================================================================================================================

/// @brief Master Commandの第1群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列を渡す.
/// @param a_flg_exe Meridimの受信成功判定フラグを渡す.
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_1(Meridim90Union a_meridim, bool a_flg_exe) {
  if (!a_flg_exe) {
    return false;
  }

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

  // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信の通信周期制御をボード側主導に（デフォルト）
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
bool execute_master_command_2(Meridim90Union a_meridim, bool a_flg_exe) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:[0] 全サーボ脱力
  if (a_meridim.sval[MRD_MASTER] == 0) {
    if (NULL != plugin_servo) {
      plugin_servo->mrd_servo_all_off(s_udp_meridim);
    }
    Serial.println("All servos torque off.");
    return true;
  }

  // コマンド:[1] サーボオン 通常動作

  // コマンド:MCMD_SENSOR_YAW_CALIB(10002) IMU/AHRSのヨー軸リセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_SENSOR_YAW_CALIB) {
    if (nullptr != plugin_ahrs) {
      if (true == plugin_ahrs->reset()) {
        Serial.println("cmd: yaw reset.");
        return true;
      }
    }
    return false;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信の通信周期制御をPC側主導に（SSH的な動作）
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
