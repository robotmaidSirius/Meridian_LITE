#ifndef __MERIDIAN_MOVEMENT_H__
#define __MERIDIAN_MOVEMENT_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

namespace meridian {
namespace app {

//==================================================================================================
//  動作計算関連の処理
//==================================================================================================

//------------------------------------------------------------------------------------
//  各動作計算モジュールへの分岐
//------------------------------------------------------------------------------------

/// @brief モーション初期化のためのスタブ関数. 現在は何も行わず, 常にfalseを返す.
/// @param a_meridim モーションデータを保持するMeridim配列.
/// @return 常にfalseを返す.
static bool mrd_move_init(Meridim90Union a_meridim) { return false; }

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

} // namespace app
} // namespace meridian

#endif // __MERIDIAN_MOVEMENT_H__
