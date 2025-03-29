#ifndef __MERIDIAN_UTILITY_H__
#define __MERIDIAN_UTILITY_H__

// ヘッダファイルの読み込み
#include <Meridim90.hpp> // Meridim90のライブラリ導入

namespace meridian {
namespace core {
namespace execution {

/// @brief 指定された位置のビットをセットする(16ビット変数用).
/// @param a_byte ビットをセットする16ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から15).
/// @return なし.
inline void mrd_setBit16(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte |= (1 << a_bit_pos); }

/// @brief 指定された位置のビットをクリアする(16ビット変数用).
/// @param a_byte ビットをクリアする16ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から15).
/// @return なし.
inline void mrd_clearBit16(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte &= ~(1 << a_bit_pos); }

/// @brief 指定された位置のビットをセットする(8ビット変数用).
/// @param value ビットをセットする8ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から7).
/// @return なし.
inline void mrd_setBit8(uint8_t &value, uint8_t a_bit_pos) { value |= (1 << a_bit_pos); }

/// @brief 指定された位置のビットをクリアする(8ビット変数用).
/// @param value ビットをクリアする8ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から7).
/// @return なし.
inline void mrd_clearBit8(uint8_t &value, uint8_t a_bit_pos) { value &= ~(1 << a_bit_pos); }

//------------------------------------------------------------------------------------
//  タイムアウト監視用タイマー
//------------------------------------------------------------------------------------
// タイマーの開始状態を保持するための静的変数
static unsigned long timeout_start = 0;
static bool flg_timer_started = false; // タイマーが開始されたかどうかのフラグ

/// @brief 指定されたミリ秒のタイムアウトを監視する. mrd_timeout_resetとセットで使う.
/// @param a_limit タイムアウトまでの時間(ms)
/// @return タイムアウトでtrueを返す.
bool mrd_timeout_check(unsigned long a_limit) {
  // タイマーが開始されていない場合は現在の時間を記録してタイマーを開始
  if (!flg_timer_started) {
    timeout_start = millis();
    flg_timer_started = true; // タイムアウト監視開始フラグをアゲる
  }

  unsigned long current_time = millis(); // 現在の時間を取得

  if (current_time - timeout_start >= a_limit) { // 指定された時間が経過しているかチェック
    flg_timer_started = false;                   // タイムアウト監視開始フラグをサゲる
    return true;                                 // 指定された時間が経過していれば true を返す
  }

  return false; // まだ時間が経過していなければ false を返す
}

/// @brief タイムアウト監視開始フラグをリセットする. mrd_timeout_checkとセットで使う.
void mrd_timeout_reset() {
  flg_timer_started = false; // タイマーをリセットして次回の呼び出しに備える
}

//------------------------------------------------------------------------------------
//  meriput / meridimへのデータ書き込み
//------------------------------------------------------------------------------------

/// @brief meridim配列のチェックサムを算出して[len-1]に書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @return 常にtrueを返す.
bool mrd_checksum(Meridim90Union &a_meridim, int len = 90) {
  int a_cksm = 0;
  for (int i = 0; i < len - 1; i++) {
    a_cksm += int(a_meridim.sval[i]);
  }
  a_meridim.sval[len - 1] = short(~a_cksm);
  return true;
}

} // namespace execution
} // namespace core
} // namespace meridian

#endif //__MERIDIAN_UTILITY_H__
