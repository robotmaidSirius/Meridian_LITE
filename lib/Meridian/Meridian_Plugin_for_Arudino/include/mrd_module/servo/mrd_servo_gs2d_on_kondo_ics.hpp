/**
 * @file mrd_servo_gs2d_on_kondo_ics.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_GS2D_ON_KONDO_ICS_HPP
#define MRD_SERVO_GS2D_ON_KONDO_ICS_HPP

// TODO: 動作未検証
// ライブラリ導入
#include "gs2d_krs.h"
#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定
#include <mrd_plugin/i_mrd_servo.hpp>

namespace meridian {
namespace modules {
namespace plugin {
class MrdServoGS2DonICS : public IMeridianServo {
private:
  const int SERVO_LOST_ERR_WAIT = 6; // 連続何フレームサーボ信号をロストしたら異常とするか
  IcsHardSerialClass ics;

public:
  MrdServoGS2DonICS() {}
  ~MrdServoGS2DonICS() {}

public:
  const char *get_name() { return "ICS(KONDO,gs2d)"; }
  bool setup() override { return true; }

  bool write(int a_id, int value) override {}
  int read(int a_id) override {}

  bool refresh(Meridim90Union &a_meridim) override { return true; }

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
  float mrd_servo_process_ics_gs2d(int a_servo_id, int a_cmnd, float a_tgt, float a_tgt_past, int a_trim, int a_cw, int &a_err_cnt, uint16_t &a_stat) {
    int val_tmp = 0;
    if (a_cmnd == 1) {
      // コマンドが1ならPos指定
      val_tmp = this->ics.setPos(a_servo_id, this->deg_to_krs(a_tgt, a_trim, a_cw));
    } else { // コマンドが0等なら脱力して値を取得
      val_tmp = this->ics.setFree(a_servo_id);
    }
    if (val_tmp == -1) { // サーボからの返信信号を受け取れなかった場合
      val_tmp = this->deg_to_krs(a_tgt_past, a_trim, a_cw);
      a_err_cnt++;
      if (a_err_cnt >= this->SERVO_LOST_ERR_WAIT) { // 一定以上の連続エラーで通信不能とみなす
        a_err_cnt = this->SERVO_LOST_ERR_WAIT;
        a_stat = 1;
      }
    } else {
      a_err_cnt = 0;
      a_stat = 0;
    }

    return this->krs_to_deg(val_tmp, a_trim, a_cw);
  }

private:
  float krs_to_deg(int krs, float trim, int cw) {
    float _x = (krs - 7500 - (trim * 29.62963)) * 0.03375 * cw;
    return _x;
  }
  int deg_to_krs(float degree, float trim, int cw) {
    float _x = 7500 + (trim * 29.6296) + (degree * 29.6296 * cw);
    if (_x > 11500) // max limit
    {
      _x = 11500;
    } else if (_x < 3500) // min limit
    {
      _x = 3500;
    }
    return static_cast<int>(_x);
  }
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // MRD_SERVO_GS2D_ON_KONDO_ICS_HPP
