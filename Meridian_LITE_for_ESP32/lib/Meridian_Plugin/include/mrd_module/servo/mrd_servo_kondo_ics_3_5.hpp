/**
 * @file mrd_servo_kondo_ics_3_5.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_KONDO_ICS_3_5_HPP
#define MRD_SERVO_KONDO_ICS_3_5_HPP

// ライブラリ導入
#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定
#include <meridian_core.hpp>
#include <mrd_plugin/i_mrd_servo.hpp>

// サーボ関連設定
#define PIN_EN_L            33      // サーボL系統のENピン
#define PIN_EN_R            4       // サーボR系統のENピン
#define SERVO_BAUDRATE_L    1250000 // L系統のICSサーボの通信速度bps
#define SERVO_BAUDRATE_R    1250000 // R系統のICSサーボの通信速度bps
#define SERVO_TIMEOUT_L     2       // L系統のICS返信待ちのタイムアウト時間
#define SERVO_TIMEOUT_R     2       // R系統のICS返信待ちのタイムアウト時間
#define SERVO_LOST_ERR_WAIT 6       // 連続何フレームサーボ信号をロストしたら異常とするか

class MrdServoKondoIcs35 : public I_Meridian_Servo {
public:
  IcsHardSerialClass *ics_L;
  IcsHardSerialClass *ics_R;

public:
  bool begin() override {
    bool a_result = true;
    if (NULL != ics_L) {
      a_result &= ics_L->begin();
    }
    if (NULL != ics_R) {
      a_result &= ics_R->begin();
    }
    return a_result;
  }

  MrdServoKondoIcs35(HardwareSerial &serial_left, HardwareSerial &serial_right) {
    this->setServoType(ServoType::KOICS3);
    this->ics_L = new IcsHardSerialClass(&serial_left, PIN_EN_L, SERVO_BAUDRATE_L, SERVO_TIMEOUT_L);
    this->ics_R = new IcsHardSerialClass(&serial_right, PIN_EN_R, SERVO_BAUDRATE_R, SERVO_TIMEOUT_R);
  }
  ~MrdServoKondoIcs35() {
  }
  bool refresh(Meridim90Union &a_meridim) override {
    // mrd_sv_drive_ics_double(a_meridim, this->sv);
    return true;
  }

  //================================================================================================
  //  KONDO ICSサーボ関連の処理
  //================================================================================================

  /// @brief ICSサーボの実行処理を行う関数
  /// @param a_servo_id サーボのインデックス番号
  /// @param a_cmd サーボのコマンド
  /// @param a_tgt サーボの目標位置
  /// @param a_tgt_past 前回のサーボの目標位置
  /// @param a_tgt_trim サーボの補正値
  /// @param a_cw サーボの回転方向補正値
  /// @param a_err_cnt サーボのエラーカウント
  /// @param a_stat サーボのステータス
  /// @param ics サーボクラスのインスタンス
  float process_ics(int a_servo_id, int a_cmd, float a_tgt, float a_tgt_past, int a_trim,
                    int a_cw, int &a_err_cnt, uint16_t &a_stat, IcsHardSerialClass &ics) {
    int val_tmp = 0;
    if (a_cmd == 1) { // コマンドが1ならPos指定
      val_tmp = ics.setPos(a_servo_id, mrd.Deg2Krs(a_tgt, a_trim, a_cw));
    } else { // コマンドが0等なら脱力して値を取得
      val_tmp = ics.setFree(a_servo_id);
    }

    if (val_tmp == -1) { // サーボからの返信信号を受け取れなかった場合
      val_tmp = mrd.Deg2Krs(a_tgt_past, a_trim, a_cw);
      a_err_cnt++;
      if (a_err_cnt >= SERVO_LOST_ERR_WAIT) { // 一定以上の連続エラーで通信不能とみなす
        a_err_cnt = SERVO_LOST_ERR_WAIT;
        a_stat = 1;
      }
    } else {
      a_err_cnt = 0;
      a_stat = 0;
    }

    return mrd.Krs2Deg(val_tmp, a_trim, a_cw);
  }

  /// @brief ICSサーボを駆動する関数
  /// @param a_meridim Meridimデータの参照
  /// @param a_sv サーボパラメータの配列
  void drive_ics_double(Meridim90Union &a_meridim, ServoParam &a_sv) {
    for (int i = 0; i < a_sv.num_max; i++) {
      // L系統サーボの処理
      if (a_sv.ixl_mount[i]) {
        a_sv.ixl_tgt[i] = this->process_ics(
            a_sv.ixl_id[i], a_meridim.sval[(i * 2) + 20], a_sv.ixl_tgt[i], a_sv.ixl_tgt_past[i],
            a_sv.ixl_trim[i], a_sv.ixl_cw[i], a_sv.ixl_err[i], a_sv.ixl_stat[i], *this->ics_L);
      }
      // R系統サーボの処理
      if (a_sv.ixr_mount[i]) {
        a_sv.ixr_tgt[i] = this->process_ics(
            a_sv.ixr_id[i], a_meridim.sval[(i * 2) + 50], a_sv.ixr_tgt[i], a_sv.ixr_tgt_past[i],
            a_sv.ixr_trim[i], a_sv.ixr_cw[i], a_sv.ixr_err[i], a_sv.ixr_stat[i], *this->ics_R);
      }
      // delayMicroseconds(2); //Teensyの場合には必要かも
    }
  }

  //================================================================================================
  //  Servo 関連の処理
  //================================================================================================

  //------------------------------------------------------------------------------------
  //  各UARTの開始
  //------------------------------------------------------------------------------------

  //------------------------------------------------------------------------------------
  //  サーボ通信フォーメーションの分岐
  //------------------------------------------------------------------------------------

  /// @brief 指定されたサーボにコマンドを分配する.
  /// @param a_meridim サーボの動作パラメータを含むMeridim配列.
  /// @param a_L_type L系統のサーボタイプ.
  /// @param a_R_type R系統のサーボタイプ.
  /// @param a_sv サーボパラメータの構造体.
  /// @return サーボの駆動が成功した場合はtrueを, 失敗した場合はfalseを返す.
  bool drive_lite(Meridim90Union &a_meridim, ServoParam &a_sv) {
    this->drive_ics_double(a_meridim, a_sv);
    return true;
  }

  //------------------------------------------------------------------------------------
  //  各種オペレーション
  //------------------------------------------------------------------------------------

  /// @brief 第一引数のMeridim配列のすべてのサーボモーターをオフ（フリー状態）に設定する.
  /// @param a_meridim サーボの動作パラメータを含むMeridim配列.
  /// @return 設定完了時にtrueを返す.
  bool all_off(Meridim90Union &a_meridim) {
    for (int i = 0; i < 15; i++) {    // 15はサーボ数
      a_meridim.sval[i * 2 + 20] = 0; // サーボのコマンドをオフに設定
      a_meridim.sval[i * 2 + 50] = 0; //
    }
    Serial.println("All servos torque off.");
    return true;
  }

  /// @brief サーボパラメータからエラーのあるサーボのインデックス番号を作る.
  /// @param a_sv サーボパラメータの構造体.
  /// @return uint8_tで番号を返す.
  ///         100-149(L系統 0-49),200-249(R系統 0-49)
  uint8_t make_errcode_lite(ServoParam a_sv) {
    uint8_t servo_ix_tmp = 0;
    for (int i = 0; i < 15; i++) {
      if (a_sv.ixl_stat[i]) {
        servo_ix_tmp = uint8_t(i + 100);
      }
      if (a_sv.ixr_stat[i]) {
        servo_ix_tmp = uint8_t(i + 200);
      }
    }
    return servo_ix_tmp;
  }
};

#endif // MRD_SERVO_KONDO_ICS_3_5_HPP
