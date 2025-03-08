#ifndef __MERIDIAN_SERVO_KONDO_ICS_H__
#define __MERIDIAN_SERVO_KONDO_ICS_H__

// ライブラリ導入
#include <IcsHardSerialClass.h>     // ICSサーボのインスタンス設定
#include <Meridim90.hpp>            // Meridim90のライブラリ導入
#include <mrd_module/sv_common.hpp> // サーボ用定義

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoKondoICS {
public:
  MrdServoKondoICS() {
  }
};

//==================================================================================================
//  KONDO ICSサーボ関連の処理
//==================================================================================================

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
float mrd_servo_process_ics(int a_servo_id, int a_cmd, float a_tgt, float a_tgt_past, int a_trim,
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
void mrd_sv_drive_ics_double(Meridim90Union &a_meridim, ServoParam &a_sv, IcsHardSerialClass &a_ics_L, IcsHardSerialClass &a_ics_R) {
  for (int i = 0; i < a_sv.num_max; i++) {
    // L系統サーボの処理
    if (a_sv.ixl.mount[i]) {
      a_sv.ixl.tgt[i] = mrd_servo_process_ics(
          a_sv.ixl.id[i], a_meridim.sval[(i * 2) + 20], a_sv.ixl.tgt[i], a_sv.ixl.tgt_past[i],
          a_sv.ixl.trim[i], a_sv.ixl.cw[i], a_sv.ixl.err[i], a_sv.ixl.stat[i], a_ics_L);
    }
    // R系統サーボの処理
    if (a_sv.ixr.mount[i]) {
      a_sv.ixr.tgt[i] = mrd_servo_process_ics(
          a_sv.ixr.id[i], a_meridim.sval[(i * 2) + 50], a_sv.ixr.tgt[i], a_sv.ixr.tgt_past[i],
          a_sv.ixr.trim[i], a_sv.ixr.cw[i], a_sv.ixr.err[i], a_sv.ixr.stat[i], a_ics_R);
    }
    // delayMicroseconds(2); //Teensyの場合には必要かも
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
bool mrd_servos_drive_lite(Meridim90Union &a_meridim, int a_L_type, int a_R_type, ServoParam &a_sv, IcsHardSerialClass &a_ics_L, IcsHardSerialClass &a_ics_R) {
  if (a_L_type == 43 && a_R_type == 43) // ICSサーボがL系R系に設定されていた場合はLR均等送信を実行
  {
    mrd_sv_drive_ics_double(a_meridim, a_sv, a_ics_L, a_ics_R);
    return true;
  } else {
    return false;
  }
}

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_SERVO_KONDO_ICS_H__
