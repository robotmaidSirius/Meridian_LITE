#ifndef __MERIDIAN_SERVO_KONDO_ICS_H__
#define __MERIDIAN_SERVO_KONDO_ICS_H__

// ライブラリ導入
#include <IcsHardSerialClass.h>     // ICSサーボのインスタンス設定
#include <Meridim90.hpp>            // Meridim90のライブラリ導入
#include <mrd_module/sv_common.hpp> // サーボ用定義

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoKondoICS : public IcsHardSerialClass {
private:
  int _index; // サーボのインデックス番号

public:
  MrdServoKondoICS(int a_index, HardwareSerial *icsSerial, byte en_pin, long baudrate, int timeout)
      : IcsHardSerialClass(icsSerial, en_pin, baudrate, timeout) {
    this->_index = a_index;
  }

  /**
   * @brief Degree value to Kondo's KRS Servo value.
   *
   * @param[in] degree Source degree value
   * @param[in] trim Trim degree value
   * @param[in] cw Correction value for direction of rotation（+1 or -1）
   * @return int, Kondo's KRS Servo value（3500-11500）
   */
  int Deg2Krs(float degree, float trim, int cw) {
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
  /**
   * @brief Kondo's KRS Servo value to degree value.
   *
   * @param[in] krs Source Kondo's KRS Servo value（3500-11500）
   * @param[in] trim Trim degree value
   * @param[in] cw Correction value for direction of rotation（+1 or -1）
   * @return float, degree
   */
  float Krs2Deg(int krs, float trim, int cw) {
    float _x = (krs - 7500 - (trim * 29.62963)) * 0.03375 * cw;
    return _x;
  }
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
  float process_ics(int a_servo_id, int a_cmd, float a_tgt, float a_tgt_past, int a_trim,
                    int a_cw, int &a_err_cnt, uint16_t &a_stat) {
    int val_tmp = 0;
    if (a_cmd == 1) { // コマンドが1ならPos指定
      val_tmp = this->setPos(a_servo_id, this->Deg2Krs(a_tgt, a_trim, a_cw));
    } else { // コマンドが0等なら脱力して値を取得
      val_tmp = this->setFree(a_servo_id);
    }

    if (val_tmp == -1) { // サーボからの返信信号を受け取れなかった場合
      val_tmp = this->Deg2Krs(a_tgt_past, a_trim, a_cw);
      a_err_cnt++;
      if (a_err_cnt >= SERVO_LOST_ERR_WAIT) { // 一定以上の連続エラーで通信不能とみなす
        a_err_cnt = SERVO_LOST_ERR_WAIT;
        a_stat = 1;
      }
    } else {
      a_err_cnt = 0;
      a_stat = 0;
    }

    return this->Krs2Deg(val_tmp, a_trim, a_cw);
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
  bool drive_lite(Meridim90Union &a_meridim, ServoParam::ServoData &a_sv_data) {
    for (int i = 0; i < a_sv_data.num_max; i++) {
      // サーボの処理
      if (a_sv_data.mount[i]) {
        a_sv_data.tgt[i] = this->process_ics(
            a_sv_data.id[i], a_meridim.sval[(i * 2) + this->_index], a_sv_data.tgt[i], a_sv_data.tgt_past[i],
            a_sv_data.trim[i], a_sv_data.cw[i], a_sv_data.err[i], a_sv_data.stat[i]);
      }
      // delayMicroseconds(2); //Teensyの場合には必要かも
    }
    return true;
  }
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_SERVO_KONDO_ICS_H__
