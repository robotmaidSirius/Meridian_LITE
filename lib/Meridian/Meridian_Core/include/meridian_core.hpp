/**
 * @file meridian_core.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MERIDIAN_CORE_HPP
#define MERIDIAN_CORE_HPP

#include "Meridim90.hpp"
#include <Meridian.h> // Meridianのライブラリ導入

extern MERIDIANFLOW::Meridian mrd;
extern Meridim90Union mrd_meridim;

/// @brief meridim配列のチェックサムを算出して[len-1]に書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @return 常にtrueを返す.
bool mrd_checksum(Meridim90Union &a_meridim, int len = MRDM_LEN);

/// @brief 配列の中で0以外が入っている最大のIndexを求める.
/// @param a_arr 配列
/// @param a_size 配列の長さ
/// @return 0以外が入っている最大のIndex. すべて0の場合は1を反す.
int mrd_max_used_index(const int a_arr[], int a_size);

/// @brief 指定された位置のビットをセットする(16ビット変数用).
/// @param a_byte ビットをセットする16ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から15).
/// @return なし.
void mrd_setBit16(uint16_t &a_byte, uint16_t a_bit_pos);

/// @brief 指定された位置のビットをクリアする(16ビット変数用).
/// @param a_byte ビットをクリアする16ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から15).
/// @return なし.
void mrd_clearBit16(uint16_t &a_byte, uint16_t a_bit_pos);

/// @brief 指定された位置のビットをセットする(8ビット変数用).
/// @param value ビットをセットする8ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から7).
/// @return なし.
void mrd_setBit8(uint8_t &value, uint8_t a_bit_pos);

/// @brief 指定された位置のビットをクリアする(8ビット変数用).
/// @param value ビットをクリアする8ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から7).
/// @return なし.
void mrd_clearBit8(uint8_t &value, uint8_t a_bit_pos);

//------------------------------------------------------------------------------------
//  タイムアウト監視用タイマー
//------------------------------------------------------------------------------------

/// @brief 指定されたミリ秒のタイムアウトを監視する. mrd_timeout_resetとセットで使う.
/// @param a_limit タイムアウトまでの時間(ms)
/// @return タイムアウトでtrueを返す.
bool mrd_timeout_check(unsigned long a_limit);

/// @brief タイムアウト監視開始フラグをリセットする. mrd_timeout_checkとセットで使う.
void mrd_timeout_reset();

#endif // MERIDIAN_CORE_HPP
