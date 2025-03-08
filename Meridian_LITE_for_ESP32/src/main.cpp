#ifndef __MERIDIAN_LITE_MAIN__
#define __MERIDIAN_LITE_MAIN__

#define VERSION "Meridian_LITE_v1.1.1_2024_08.18" // バージョン表示

/// @file    Meridian_LITE_for_ESP32/src/main.cpp
/// @brief   Meridian is a system that smartly realizes the digital twin of a robot.
/// @details Meridian_LITE for Meridian Board -LITE- with ESP32DecKitC.
///
/// This code is licensed under the MIT License.
/// Copyright (c) 2022 Izumi Ninagawa & Project Meridian

//==================================================================================================
//  初期設定
//==================================================================================================

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"

#include "application/mrd_app.hpp"
#include "mrd_bt_pad.h"
#include "mrd_disp.h"
#include "mrd_move.h"
#include "mrd_servo.h"
#include "mrd_util.h"
#include "mrd_wifi.h"
#include "mrd_wire0.h"
#include <Meridim90.hpp>
#include <meridian_core_for_arduino.hpp>

// ライブラリ導入
#include <Arduino.h>
#include <Meridian.h> // Meridianのライブラリ導入

using namespace meridian::core::execution;
using namespace meridian::core::communication;
using namespace meridian::modules::config;
using namespace meridian::modules::plugin;
using namespace meridian::app;

MrdConversation mrd_wifi(WIFI_SEND_IP, UDP_SEND_PORT, UDP_RESV_PORT);

Meridim90Union s_udp_meridim;       // Meridim配列データ送信用(short型, センサや角度は100倍値)
Meridim90Union r_udp_meridim;       // Meridim配列データ受信用
Meridim90Union s_udp_meridim_dummy; // SPI送信ダミー用

MERIDIANFLOW::Meridian mrd;
MrdServoKondoICS mrd_servo_l(20, &Serial1, PIN_EN_L, SERVO_BAUDRATE_L, SERVO_TIMEOUT_L);
MrdServoKondoICS mrd_servo_r(50, &Serial2, PIN_EN_R, SERVO_BAUDRATE_R, SERVO_TIMEOUT_R);

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

// シーケンス番号理用の変数
struct MrdSq {
  int s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
  int r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};
// モニタリング設定
struct MrdMonitor {
  bool flow = MONITOR_FLOW;           // フローを表示
  bool all_err = MONITOR_ERR_ALL;     // 全経路の受信エラー率を表示
  bool servo_err = MONITOR_ERR_SERVO; // サーボエラーを表示
  bool seq_num = MONITOR_SEQ;         // シーケンス番号チェックを表示
  bool pad = MONITOR_PAD;             // リモコンのデータを表示
};
// フラグ用変数
struct MrdFlags {
  bool udp_board_passive = false; // UDP通信の周期制御がボード主導(false) か, PC主導(true)か.

  bool stop_board_during = false; // ボードの末端処理をmeridim[2]秒, meridim[3]ミリ秒だけ止める.
  bool sdcard_write_mode = false; // SDCARDへの書き込みモード.
  bool sdcard_read_mode = false;  // SDCARDからの読み込みモード.
  bool wire0_init = false;        // I2C 0系統の初期化合否
  bool wire1_init = false;        // I2C 1系統の初期化合否
  bool bt_busy = false;           // Bluetoothの受信中フラグ（UDPコンフリクト回避用）
  bool spi_rcvd = true;           // SPIのデータ受信判定
  bool udp_rcvd = false;          // UDPのデータ受信判定
  bool udp_busy = false;          // UDPスレッドでの受信中フラグ（送信抑制）

  bool udp_receive_mode = NETWORK_UDP_RECEIVE; // PCからのデータ受信実施（0:OFF, 1:ON, 通常は1）
  bool udp_send_mode = NETWORK_UDP_SEND;       // PCへのデータ送信実施（0:OFF, 1:ON, 通常は1）
  bool meridim_rcvd = false;                   // Meridimが正しく受信できたか.
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

//------------------------------------------------------------------------------------
//  変数
//------------------------------------------------------------------------------------

// システム用の変数
TaskHandle_t thp[4]; // マルチスレッドのタスクハンドル格納用

// 管理用変数
MrdFlags flg;
MrdSq mrdsq;
MrdTimer tmr;
MrdErr err;
PadUnion pad_array = {0}; // pad値の格納用配列
PadUnion pad_i2c = {0};   // pad値のi2c送受信用配列
PadValue pad_analog;
AhrsValue ahrs;
ServoParam sv;
MrdMonitor monitor;

meridian::core::communication::MrdMsgHandler mrd_disp(Serial);

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
//==================================================================================================
//  プロテクト宣言
//==================================================================================================
// 予約用
bool execute_master_command_1(Meridim90Union a_meridim, bool a_flg_exe);
bool execute_master_command_2(Meridim90Union a_meridim, bool a_flg_exe);

//==================================================================================================
//  SETUP
//==================================================================================================
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
  mrd_disp.charging(CHARGE_TIME_MS);

  // 起動メッセージの表示(バージョン, PC-USB,SPI0,i2c0のスピード)
  mrd_disp.hello_lite_esp(VERSION, SERIAL_PC_BPS, SPI0_SPEED, IMUAHRS_I2C0_SPEED);

  // サーボ値の初期設定
  app_servo_setup(sv);

  // サーボUARTの通信速度の表示
  mrd_disp.servo_bps_2lines(SERVO_BAUDRATE_L, SERVO_BAUDRATE_R);

  // サーボ用UART設定
  mrd_servo_l.begin(); // サーボモータの通信初期設定. Serial2
  mrd_servo_r.begin(); // サーボモータの通信初期設定. Serial3

  mrd_disp.servo_protocol(MrdMsgHandler::UartLine::L, SERVO_MOUNT_TYPE_L); // サーボプロトコルの表示
  mrd_disp.servo_protocol(MrdMsgHandler::UartLine::R, SERVO_MOUNT_TYPE_R);

  // マウントされたサーボIDの表示
  mrd_disp.servo_mounts_2lines(sv);

  // EEPROMの開始, ダンプ表示
  app_eeprom_setup();

  // SDカードの初期設定とチェック
  app_sd_setup();

  // I2Cの初期化と開始
  mrd_wire0_setup(IMUAHRS_MOUNT, IMUAHRS_I2C0_SPEED, ahrs, PIN_I2C0_SDA, PIN_I2C0_SCL);

  // I2Cスレッドの開始
  if (IMUAHRS_MOUNT == BNO055_AHRS) {
    xTaskCreatePinnedToCore(mrd_wire0_Core0_bno055_r, "Core0_bno055_r", 4096, NULL, 2, &thp[0], 0);
    Serial.println("Core0 thread for BNO055 start.");
    delay(10);
  }

  // WiFiの初期化と開始
  mrd_disp.esp_wifi(WIFI_AP_SSID);
  if (mrd_wifi.init(WIFI_AP_SSID, WIFI_AP_PASS, Serial)) {
    mrd_wifi.enable_send(NETWORK_UDP_SEND);
    mrd_wifi.enable_receive(NETWORK_UDP_RECEIVE);
    // wifiIPの表示
    mrd_disp.esp_ip(NETWORK_FIXED_IP, WIFI_SEND_IP, FIXED_IP_ADDR);
  }

  // コントロールパッドの種類を表示
  mrd_disp.mounted_pad(PAD_MOUNT);

  // Bluetoothの開始と表示(WIIMOTE)
  if (PAD_MOUNT == WIIMOTE) { // Bluetooth用スレッドの開始
    mrd_bt_settings(PAD_MOUNT, PAD_INIT_TIMEOUT, wiimote, PIN_LED_BT, Serial);
    xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 2048, NULL, 5, &thp[2], 0);
  }

  // UDP開始用ダミーデータの生成
  s_udp_meridim.sval[MRD_MASTER] = 90;
  s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
  r_udp_meridim.sval[MRD_MASTER] = 90;
  r_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(r_udp_meridim.sval, MRDM_LEN);

  // タイマーの設定
  mrd_timer_setup(FRAME_DURATION);
  mrd_disp.flow_start_lite_esp(); // 開始メッセージ
}

//==================================================================================================
// MAIN LOOP
//==================================================================================================
void loop() {

  //------------------------------------------------------------------------------------
  //  [ 1 ] UDP送信
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[1]", monitor.flow); // デバグ用フロー表示

  // @[1-1] UDP送信の実行
  if (flg.udp_send_mode) // UDPの送信実施フラグの確認（モード確認）
  {
    flg.udp_busy = true; // UDP使用中フラグをアゲる
    mrd_wifi.udp_send(s_udp_meridim.bval, MRDM_BYTE);
    flg.udp_busy = false; // UDP使用中フラグをサゲる
    flg.udp_rcvd = false; // UDP受信完了フラグをサゲる
  }

  //------------------------------------------------------------------------------------
  //  [ 2 ] UDP受信
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[2]", monitor.flow); // デバグ用フロー表示

  // @[2-1] UDPの受信待ち受けループ
  if (flg.udp_receive_mode) // UDPの受信実施フラグの確認（モード確認）
  {
    unsigned long start_tmp = millis();
    flg.udp_busy = true;  // UDP使用中フラグをアゲる
    flg.udp_rcvd = false; // UDP受信完了フラグをサゲる
    while (!flg.udp_rcvd) {
      // UDP受信処理
      if (mrd_wifi.udp_receive(r_udp_meridim.bval, MRDM_BYTE)) // 受信確認
      {
        flg.udp_rcvd = true; // UDP受信完了フラグをアゲる
      }

      // タイムアウト抜け処理
      unsigned long current_tmp = millis();
      if (current_tmp - start_tmp >= NETWORK_UDP_TIMEOUT) {
        if (millis() > MONITOR_SUPPRESS_DURATION) { // 起動直後はエラー表示を抑制
          Serial.printf("UDP timeout[%d]\n", mrdsq.s_increment);
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
    mrd_disp.monitor_check_flow("CsOK", monitor.flow); // デバグ用フロー表示

    // @[2-3] UDP受信配列から UDP送信配列にデータを転写
    memcpy(s_udp_meridim.bval, r_udp_meridim.bval, MRDM_LEN * 2);

    // @[2-4a] エラービット14番(ESP32のPCからのUDP受信エラー検出)をサゲる
    mrd_clearBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);

  } else // チェックサムがNGならバッファから転記せず前回のデータを使用する
  {

    // @[2-4b] エラービット14番(ESP32のPCからのUDP受信エラー検出)をアゲる
    mrd_setBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);
    err.pc_esp++;
    mrd_disp.monitor_check_flow("CsErr*", monitor.flow); // デバグ用フロー表示
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
  mrd_disp.monitor_check_flow("[3]", monitor.flow); // デバグ用フロー表示

  // @[3-1] MasterCommand group1 の処理
  execute_master_command_1(s_udp_meridim, flg.meridim_rcvd);

  //------------------------------------------------------------------------------------
  //  [ 4 ] センサー類読み取り
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[4]", monitor.flow); // デバグ用フロー表示

  // @[4-1] センサ値のMeridimへの転記
  meriput90_ahrs(s_udp_meridim, ahrs.read, IMUAHRS_MOUNT); // BNO055_AHRS

  //------------------------------------------------------------------------------------
  //  [ 5 ] リモコンの読み取り
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[5]", monitor.flow); // デバグ用フロー表示

  // @[5-1] リモコンデータの書き込み
  if (PAD_MOUNT > 0) { // リモコンがマウントされていれば

    // リモコンデータの読み込み
    pad_array.ui64val = mrd_pad_read(PAD_MOUNT, pad_array.ui64val, mrd_servo_r);

    // リモコンの値をmeridimに格納する
    meriput90_pad(s_udp_meridim, pad_array, PAD_BUTTON_MARGE);
  }

  //------------------------------------------------------------------------------------
  //  [ 6 ] MasterCommand group2 の処理
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[6]", monitor.flow); // デバグ用フロー表示

  // @[6-1] MasterCommand group2 の処理
  execute_master_command_2(s_udp_meridim, flg.meridim_rcvd);

  //------------------------------------------------------------------------------------
  //  [ 7 ] ESP32内部で位置制御する場合の処理
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[7]", monitor.flow); // デバグ用フロー表示

  // @[7-1] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
  for (int i = 0; i <= sv.ixl.num_max; i++) {
    sv.ixl.tgt_past[i] = sv.ixl.tgt[i];                    // 前回のdegreeをキープ
    sv.ixl.tgt[i] = s_udp_meridim.sval[i * 2 + 21] * 0.01; // 受信したdegreeを格納
  }
  for (int i = 0; i <= sv.ixr.num_max; i++) {
    sv.ixr.tgt_past[i] = sv.ixr.tgt[i];                    // 前回のdegreeをキープ
    sv.ixr.tgt[i] = s_udp_meridim.sval[i * 2 + 51] * 0.01; // 受信したdegreeを格納
  }

  // @[7-2] ESP32による次回動作の計算
  // 以下はリモコンの左十字キー左右でL系統0番サーボ（首部）を30度左右にふるサンプル
  if (s_udp_meridim.sval[MRD_PAD_BUTTONS] == PAD_RIGHT) {
    sv.ixl.tgt[0] = -30.00; // -30度
  } else if (s_udp_meridim.sval[MRD_PAD_BUTTONS] == PAD_LEFT) {
    sv.ixl.tgt[0] = 30.00; // +30度
  }

  // @[7-3] 各種処理

  //------------------------------------------------------------------------------------
  //  [ 8 ] サーボ動作の実行
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[8]", monitor.flow); // デバグ用フロー表示

  // @[8-1] サーボ受信値の処理
  if (!MODE_ESP32_STANDALONE) {                    // サーボ処理を行うかどうか
    mrd_servo_l.drive_lite(s_udp_meridim, sv.ixl); // サーボ動作を実行する
    mrd_servo_r.drive_lite(s_udp_meridim, sv.ixr); // サーボ動作を実行する
  } else {
    // ボード単体動作モードの場合はサーボ処理をせずL0番サーボ値として+-30度のサインカーブ値を返す
    sv.ixl.tgt[0] = sin(tmr.count_loop * M_PI / 180.0) * 30;
  }

  //------------------------------------------------------------------------------------
  //  [ 9 ] サーボ受信値の処理
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[9]", monitor.flow); // デバグ用フロー表示

  // @[9-1] サーボIDごとにの現在位置もしくは計算結果を配列に格納
  for (int i = 0; i <= sv.ixl.num_max; i++) {
    // 最新のサーボ角度をdegreeで格納
    s_udp_meridim.sval[i * 2 + 21] = mrd.float2HfShort(sv.ixl.tgt[i]);
  }
  for (int i = 0; i <= sv.ixr.num_max; i++) {
    // 最新のサーボ角度をdegreeで格納
    s_udp_meridim.sval[i * 2 + 51] = mrd.float2HfShort(sv.ixr.tgt[i]);
  }

  //------------------------------------------------------------------------------------
  //  [ 10 ] エラーリポートの作成
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[10]", monitor.flow); // デバグ用フロー表示

  // @[10-1] エラーリポートの表示
  // mrd_msg_all_err(err, monitor.all_err);
  mrd_disp.all_err(MONITOR_ERR_ALL, err);

  //------------------------------------------------------------------------------------
  //  [ 11 ] UDP送信信号作成
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[11]", monitor.flow); // デバグ用フロー表示

  // @[11-1] フレームスキップ検出用のカウントをカウントアップして送信用に格納
  mrdsq.s_increment = mrd.seq_increase_num(mrdsq.s_increment);
  s_udp_meridim.usval[1] = mrdsq.s_increment;

  // @[11-2] エラーが出たサーボのインデックス番号を格納
  s_udp_meridim.ubval[MRD_ERR_l] = app_servo_make_errcode_lite(sv);

  // @[11-3] チェックサムを計算して格納
  // s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
  mrd_meriput90_cksm(s_udp_meridim);

  //------------------------------------------------------------------------------------
  //   [ 12 ] フレーム終端処理
  //------------------------------------------------------------------------------------
  mrd_disp.monitor_check_flow("[12]", monitor.flow); // 動作チェック用シリアル表示

  // @[12-1] count_timerがcount_frameに追いつくまで待機
  mrd_timer_delay();

  mrd_disp.monitor_check_flow("\n", monitor.flow); // 動作チェック用シリアル表示
}

//==================================================================================================
//  コマンド処理
//==================================================================================================

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
    app_servo_err_reset(sv);
    Serial.println("Servo Error ID reset.");
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信の通信周期制御をボード側主導に（デフォルト）
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_ACTIVE) {
    flg.udp_board_passive = false; // UDP送信をアクティブモードに
    mrd_timer_clear();             // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_EEPROM_ENTER_WRITE (10009) EEPROMの書き込みモードスタート
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_ENTER_WRITE) {
    mrd_set_eeprom();
    mrd_timer_clear(); // フレームの管理時計をリセットフラグをセット
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
    app_servo_all_off(s_udp_meridim);
    return true;
  }

  // コマンド:[1] サーボオン 通常動作

  // コマンド:MCMD_SENSOR_YAW_CALIB(10002) IMU/AHRSのヨー軸リセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_SENSOR_YAW_CALIB) {
    ahrs.yaw_origin = ahrs.yaw_source;
    Serial.println("cmd: yaw reset.");
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信の通信周期制御をPC側主導に（SSH的な動作）
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_PASSIVE) {
    flg.udp_board_passive = true; // UDP送信をパッシブモードに
    mrd_timer_clear();            // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_FRAMETIMER_RESET) (10007) フレームカウンタを現在時刻にリセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_FRAMETIMER_RESET) {
    mrd_timer_clear(); // フレームの管理時計をリセットフラグをセット
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
    mrd_timer_clear();             // フレームの管理時計をリセットフラグをセット
    return true;
  }
  return false;
}

#endif // __MERIDIAN_LITE_MAIN__
