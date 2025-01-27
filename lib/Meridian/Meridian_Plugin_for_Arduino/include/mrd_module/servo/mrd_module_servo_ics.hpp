/**
 * @file mrd_module_servo_ics.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_SERVO_ICS_HPP__
#define __MRD_MODULE_SERVO_ICS_HPP__

// ヘッダーファイルの読み込み
#include <mrd_modules/mrd_plugin/i_mrd_plugin_servo.hpp>

// ライブラリ導入

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoICS : public IMeridianServo {
public:
  MrdServoICS() {}
  ~MrdServoICS() {}

public:
  const char *get_name() override { return "ICS3.5/3.6(KONDO,KRS)"; };
  bool setup() override { return true; };
  bool input(Meridim90 &a_meridim) override { return true; };
  bool output(Meridim90 &a_meridim) override { return true; };

public:
  //================================================================================================================
  //  Servo 関連の処理
  //================================================================================================================

  //------------------------------------------------------------------------------------
  //  各UARTの開始
  //------------------------------------------------------------------------------------

  /// @brief 指定されたUARTラインとサーボタイプに基づいてサーボの通信プロトコルを設定する.
  /// @param a_line UART通信ライン(L, R, またはC).
  /// @param a_servo_type サーボのタイプを示す整数値.
  /// @return サーボがサポートされている場合はtrueを, サポートされていない場合はfalseを返す.
  bool mrd_servo_begin(UartLine a_line, int a_servo_type) {
    switch (a_servo_type) {
    case 1:
      // single PWM [WIP]
      return false;
    case 11:
      // I2C_PCA9685 to PWM [WIP]
      return false;
    case 21:
      // RSxTTL (FUTABA) [WIP]
      return false;
    case 31:
      // DYNAMIXEL Protocol 1.0 [WIP]
      return false;
    case 32:
      // DYNAMIXEL Protocol 2.0 [WIP]
      return false;
    case 43:
      if (a_line == UartLine::L)
        ics_L.begin(); // サーボモータの通信初期設定. Serial2
      else if (a_line == UartLine::R)
        ics_R.begin(); // サーボモータの通信初期設定. Serial3
      return true;
    case 44:
      // PMX(KONDO) [WIP]
      return false;
    case 51:
      // XBUS(JR PROPO) [WIP]
      return false;
    case 61:
      // STS(FEETECH) [WIP]
      return false;
    case 62:
      // SCS(FEETECH) [WIP]
      return false;
    default:
      // Not defined.
      return false;
    }
  }

  //------------------------------------------------------------------------------------
  //  サーボ通信フォーメーションの分岐
  //------------------------------------------------------------------------------------

  /// @brief 指定されたサーボにコマンドを分配する.
  /// @param a_meridim サーボの動作パラメータを含むMeridim配列.
  /// @param a_L_type L系統のサーボタイプ.
  /// @param a_R_type R系統のサーボタイプ.
  /// @param a_sv サーボパラメータの構造体.
  /// @return サーボの駆動が成功した場合はtrueを, 失敗した場合はfalseを返す.
  bool mrd_servos_drive_lite(Meridim90Union &a_meridim, int a_L_type, int a_R_type, ServoParam &a_sv) {
    if (a_L_type == 43 && a_R_type == 43) // ICSサーボがL系R系に設定されていた場合はLR均等送信を実行
    {
      mrd_sv_drive_ics_double(a_meridim, a_sv);
      return true;
    } else {
      return false;
    }
  }

  //------------------------------------------------------------------------------------
  //  各種オペレーション
  //------------------------------------------------------------------------------------

  /// @brief 第一引数のMeridim配列のすべてのサーボモーターをオフ(フリー状態)に設定する.
  /// @param a_meridim サーボの動作パラメータを含むMeridim配列.
  /// @return 設定完了時にtrueを返す.
  bool mrd_servo_all_off(Meridim90Union &a_meridim) {
    for (int i = 0; i < 15; i++) {    // 15はサーボ数
      a_meridim.sval[i * 2 + 20] = 0; // サーボのコマンドをオフに設定
      a_meridim.sval[i * 2 + 50] = 0; //
    }
    Serial.println("All servos torq off.");
    return true;
  }

  /// @brief サーボパラメータからエラーのあるサーボのインデックス番号を作る.
  /// @param a_sv サーボパラメータの構造体.
  /// @return uint8_tで番号を返す.
  ///         100-149(L系統 0-49),200-249(R系統 0-49)
  uint8_t mrd_servos_make_errcode_lite(ServoParam a_sv) {
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

  //================================================================================================================
  //  KONDO ICSサーボ関連の処理
  //================================================================================================================

  /// @brief ICSサーボの実行処理を行う関数
  /// @param a_servo_id サーボのインデックス番号
  /// @param a_cmnd サーボのコマンド
  /// @param a_tgt サーボの目標位置
  /// @param a_tgt_past 前回のサーボの目標位置
  /// @param a_tgt_trim サーボの補正値
  /// @param a_cw サーボの回転方向補正値
  /// @param a_err_cnt サーボのエラーカウント
  /// @param a_stat サーボのステータス
  /// @param ics サーボクラスのインスタンス
  float mrd_servo_process_ics(int a_servo_id, //
                              int a_cmnd, float a_tgt, float a_tgt_past, int a_trim,
                              int a_cw, int &a_err_cnt, uint16_t &a_stat, IcsHardSerialClass &ics) {
    int val_tmp = 0;
    if (a_cmnd == 1) { // コマンドが1ならPos指定
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
  void mrd_sv_drive_ics_double(Meridim90Union &a_meridim, ServoParam &a_sv) {
    for (int i = 0; i < a_sv.num_max; i++) {
      // L系統サーボの処理
      if (a_sv.ixl_mount[i]) {
        a_sv.ixl_tgt[i] = mrd_servo_process_ics(
            a_sv.ixl_id[i], a_meridim.sval[(i * 2) + 20], a_sv.ixl_tgt[i], a_sv.ixl_tgt_past[i],
            a_sv.ixl_trim[i], a_sv.ixl_cw[i], a_sv.ixl_err[i], a_sv.ixl_stat[i], ics_L);
      }
      // R系統サーボの処理
      if (a_sv.ixr_mount[i]) {
        a_sv.ixr_tgt[i] = mrd_servo_process_ics(
            a_sv.ixr_id[i], a_meridim.sval[(i * 2) + 50], a_sv.ixr_tgt[i], a_sv.ixr_tgt_past[i],
            a_sv.ixr_trim[i], a_sv.ixr_cw[i], a_sv.ixr_err[i], a_sv.ixr_stat[i], ics_R);
      }
      // delayMicroseconds(2); //Teensyの場合には必要かも
    }
  }
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_SERVO_ICS_HPP__

#if 0
        m_serial.print("single PWM");
        m_serial.print("I2C_PCA9685 to PWM");
        m_serial.print("RSxTTL (FUTABA)");
        m_serial.print("DYNAMIXEL Protocol 1.0");
        m_serial.print("DYNAMIXEL Protocol 2.0");
        m_serial.println("ICS3.5/3.6(KONDO,KRS)");
        m_serial.print("PMX(KONDO)");
        m_serial.print("XBUS(JR PROPO)");
        m_serial.print("STS(FEETECH)");
        m_serial.print("SCS(FEETECH)");
#endif

#if 1
#include "gs2d_krs.h"

//================================================================================================================
//  gs2d によるICSサーボの処理
//  https://github.com/karakuri-products/gs2d
//================================================================================================================

/// @brief ICSサーボの実行処理を行う関数
/// @param a_servo_id サーボのインデックス番号
/// @param a_cmnd サーボのコマンド
/// @param a_tgt サーボの目標位置
/// @param a_tgt_past 前回のサーボの目標位置
/// @param a_tgt_trim サーボの補正値
/// @param a_cw サーボの回転方向補正値
/// @param a_err_cnt サーボのエラーカウント
/// @param a_stat サーボのステータス
/// @param ics サーボクラスのインスタンス
float mrd_servo_process_ics_gs2d(int a_servo_id, int a_cmnd, float a_tgt, float a_tgt_past, int a_trim,
                                 int a_cw, int &a_err_cnt, uint16_t &a_stat, IcsHardSerialClass &ics) {
  int val_tmp = 0;
  if (a_cmnd == 1) { // コマンドが1ならPos指定
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

#endif
