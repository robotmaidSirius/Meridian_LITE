/**
 * @file meridian_core.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MERIDIAN_CORE_HPP__
#define __MERIDIAN_CORE_HPP__

#include "Meridim90.hpp"

namespace meridian {
namespace core {
namespace execution {
using namespace meridian::core::meridim;

void mrd_convert_array(uint8_t *data, int len, Meridim90 &a_meridim);
void mrd_convert_Meridim90(Meridim90 &a_meridim, const uint8_t *data, int len);
void meridim_countup(Meridim90 &a_meridim);
void meridim_clear(Meridim90 &a_meridim);

/// @brief meridim配列のチェックサムを算出して[len-1]に書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
void mrd_set_checksum(Meridim90 &a_meridim);

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

} // namespace execution
} // namespace core
} // namespace meridian

#endif // __MERIDIAN_CORE_HPP__
