#ifndef __MERIDIAN_SERVO_KONDO_ICS_H__
#define __MERIDIAN_SERVO_KONDO_ICS_H__

// ヘッダファイルの読み込み
#include "sv_common.hpp" // サーボ用定義

// ライブラリ導入
#include "gs2d_krs.h"
#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定

namespace meridian {
namespace modules {
namespace plugin {

//==================================================================================================
//  gs2d によるICSサーボの処理
//  https://github.com/karakuri-products/gs2d
//==================================================================================================

class MrdServoGS2dKrs {
public:
  MrdServoGS2dKrs() {
  }
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
  float mrd_servo_process_ics_gs2d(int a_servo_id, int a_cmd,
                                   float a_tgt, float a_tgt_past, int a_trim,
                                   int a_cw, int &a_err_cnt, uint16_t &a_stat,
                                   IcsHardSerialClass &ics) {
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
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_SERVO_KONDO_ICS_H__
