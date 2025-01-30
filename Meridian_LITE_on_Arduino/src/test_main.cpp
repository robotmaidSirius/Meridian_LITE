#ifndef __MERIDIAN_LITE_MAIN__
#define __MERIDIAN_LITE_MAIN__

#define VERSION "Meridian_LITE_v1.1.1_2024_08.18" // バージョン表示

#include <board/meridian_board_lite.hpp>

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"
#include "main.h"

#include "mrd_communication/mrd_conversation_wifi.hpp"
#include "mrd_communication/mrd_disp.h"
#include "mrd_module/filesystem/mrd_module_eeprom.hpp"
#include "mrd_module/filesystem/mrd_module_sd.hpp"
#include "mrd_module/pad/mrd_bt_pad.h"
#include "mrd_module/servo/mrd_module_servo_ics.hpp"

// ライブラリ導入
#include <Arduino.h>

//================================================================================================================
//================================================================================================================
// シリアルモニタリング
#define MONITOR_FRAME_DELAY       1    // シリアルモニタでフレーム遅延時間を表示(0:OFF, 1:ON)
#define MONITOR_FLOW              0    // シリアルモニタでフローを表示(0:OFF, 1:ON)
#define MONITOR_ERR_SERVO         0    // シリアルモニタでサーボエラーを表示(0:OFF, 1:ON)
#define MONITOR_ERR_ALL           0    // 全経路の受信エラー率を表示
#define MONITOR_SEQ               0    // シリアルモニタでシーケンス番号チェックを表示(0:OFF, 1:ON)
#define MONITOR_PAD               0    // シリアルモニタでリモコンのデータを表示(0:OFF, 1:ON)
#define MONITOR_SUPPRESS_DURATION 8000 // 起動直後のタイムアウトメッセージ抑制時間(単位ms)

//================================================================================================================

// モニタリング設定
struct MrdMonitor {
  bool flow = MONITOR_FLOW;           // フローを表示
  bool all_err = MONITOR_ERR_ALL;     // 全経路の受信エラー率を表示
  bool servo_err = MONITOR_ERR_SERVO; // サーボエラーを表示
  bool seq_num = MONITOR_SEQ;         // シーケンス番号チェックを表示
  bool pad = MONITOR_PAD;             // リモコンのデータを表示
};
MrdMonitor monitor;

//================================================================================================================
MERIDIANFLOW::Meridian mrd;

//================================================================================================================
//  SETUP
//================================================================================================================
MrdEEPROM _eeprom(540);
MrdSdCard _sd_card(5);
MrdConversationWifi _comm;
MrdServoICS *_servo_l;
MrdServoICS *_servo_r;

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
  // ピンモードの設定
  pinMode(PIN_ERR_LED, OUTPUT); // エラー通知用LED

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
  _servo_l->setup(); // サーボモータの通信初期設定. Serial2 Serial3

  // マウントされたサーボIDの表示
  mrd_disp.servo_mounts_2lines(sv);

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

  _eeprom.setup();

  start_pad();

  // UDP開始用ダミーデータの生成
  s_udp_meridim.sval[MRD_MASTER] = 90;
  s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
  r_udp_meridim.sval[MRD_MASTER] = 90;
  r_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(r_udp_meridim.sval, MRDM_LEN);

  mrd_timer_setup(10); // タイマーの設定
}
//------------------------------------------------------------------------------------
//  meriput / meridimへのデータ書き込み
//------------------------------------------------------------------------------------

/// @brief meridim配列のチェックサムを算出して[len-1]に書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @return 常にtrueを返す.
bool mrd_meriput90_cksm(Meridim90Union &a_meridim, int len = 90) {
  int a_cksm = 0;
  for (int i = 0; i < len - 1; i++) {
    a_cksm += int(a_meridim.sval[i]);
  }
  a_meridim.sval[len - 1] = short(~a_cksm);
  return true;
}

//================================================================================================================
// MAIN LOOP
//================================================================================================================
void test_loop() {

  //------------------------------------------------------------------------------------
  //  [ 2 ] UDP受信
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[2]", monitor.flow); // デバグ用フロー表示

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
    mrd.monitor_check_flow("CsOK", monitor.flow); // デバグ用フロー表示

    // @[2-3] UDP受信配列から UDP送信配列にデータを転写
    memcpy(s_udp_meridim.bval, r_udp_meridim.bval, MRDM_LEN * 2);

    // @[2-4a] エラービット14番(ESP32のPCからのUDP受信エラー検出)をサゲる
    mrd_clearBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_PC_ESP);

  } else { // チェックサムがNGならバッファから転記せず前回のデータを使用する

    // @[2-4b] エラービット14番(ESP32のPCからのUDP受信エラー検出)をアゲる
    mrd_setBit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_PC_ESP);
    err.pc_esp++;
    mrd.monitor_check_flow("CsErr*", monitor.flow); // デバグ用フロー表示
  }

  // @[2-5] シーケンス番号チェック
  mrdsq.r_expect = mrd_seq_predict_num(mrdsq.r_expect); // シーケンス番号予想値の生成

  // @[2-6] シーケンス番号のシリアルモニタ表示
  mrd_disp.seq_number(mrdsq.r_expect, r_udp_meridim.usval[MRD_SEQ], monitor.seq_num);

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

  //------------------------------------------------------------------------------------
  //  [ 3 ] MasterCommand group1 の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[3]", monitor.flow); // デバグ用フロー表示

  // @[3-1] MasterCommand group1 の処理
  execute_master_command_1(s_udp_meridim, flg.meridim_rcvd);

  //------------------------------------------------------------------------------------
  //  [ 5 ] リモコンの読み取り
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[5]", monitor.flow); // デバグ用フロー表示

  // @[5-1] リモコンデータの書き込み
  if (MOUNT_PAD > 0) { // リモコンがマウントされていれば

    // リモコンデータの読み込み
    pad_array.ui64val = mrd_pad_read(MOUNT_PAD, pad_array.ui64val);

#if 0
// リモコンの値をmeridimに格納する
    // 引数の修正が必要
    meriput90_pad(s_udp_meridim, pad_array, PAD_BUTTON_MARGE);
#endif
  }

  //------------------------------------------------------------------------------------
  //  [ 6 ] MastarCommand group2 の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[6]", monitor.flow); // デバグ用フロー表示

  // @[6-1] MastarCommand group2 の処理
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
  // 以下はリモコンの左十字キー左右でL系統0番サーボ(首部)を30度左右にふるサンプル
  if (s_udp_meridim.sval[MRD_PAD_BUTTONS] == PAD_RIGHT) {
    sv.ixl_tgt[0] = -30.00; // -30度
  } else if (s_udp_meridim.sval[MRD_PAD_BUTTONS] == PAD_LEFT) {
    sv.ixl_tgt[0] = 30.00; // +30度
  }

  // @[7-3] 各種処理

  //------------------------------------------------------------------------------------
  //  [ 8 ] サーボ動作の実行
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[8]", monitor.flow); // デバグ用フロー表示

  // @[8-1] サーボ受信値の処理
  if (!MODE_ESP32_STDALONE) {          // サーボ処理を行うかどうか
    _servo_l->mrd_servos_drive_lite(); // サーボ動作を実行する
    _servo_r->mrd_servos_drive_lite(); // サーボ動作を実行する
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
  s_udp_meridim.ubval[MRD_ERR_l] = _servo_l->make_errcode_lite();
  s_udp_meridim.ubval[MRD_ERR_l] = _servo_r->make_errcode_lite();

  // @[11-3] チェックサムを計算して格納
  // s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
  mrd_meriput90_cksm(s_udp_meridim);

  //------------------------------------------------------------------------------------
  //   [ 12 ] フレーム終端処理
  //------------------------------------------------------------------------------------

  // @[12-1] count_timerがcount_frameに追いつくまで待機
  mrd_timer_delay();

  // @[12-2] 必要に応じてフレームの遅延累積時間frameDelayをリセット
  if (flg.count_frame_reset) {
    mrd_timer_clear();
  }
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
bool execute_master_command_2(Meridim90Union a_meridim, bool a_flg_exe) {
  if (!a_flg_exe) {
    return false;
  }
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

#endif // __MERIDIAN_LITE_MAIN__
