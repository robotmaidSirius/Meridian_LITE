/**
 * @file mrd_module_pad_krc5fh.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_PAD_KRC5FH_HPP__
#define __MRD_MODULE_PAD_KRC5FH_HPP__

// ヘッダーファイルの読み込み
#include <mrd_module/mrd_plugin/i_mrd_plugin_pad.hpp>

// ライブラリ導入

//================================================================================================================
//  サーボIDとロボット部位、軸との対応表 (KHR-3HVの例)
//================================================================================================================
// ID    Parts/Axis ＜ICS_Left_Upper SIO1,SIO2＞
// [L00] 頭/ヨー
// [L01] 左肩/ピッチ
// [L02] 左肩/ロール
// [L03] 左肘/ヨー
// [L04] 左肘/ピッチ
// [L05] -
// ID    Parts/Axis ＜ICS_Left_Lower SIO3,SIO4＞
// [L06] 左股/ロール
// [L07] 左股/ピッチ
// [L08] 左膝/ピッチ
// [L09] 左足首/ピッチ
// [L10] 左足首/ロール
// ID    Parts/Axis  ＜ICS_Right_Upper SIO5,SIO6＞
// [R00] 腰/ヨー
// [R01] 右肩/ピッチ
// [R02] 右肩/ロール
// [R03] 右肘/ヨー
// [R04] 右肘/ピッチ
// [R05] -
// ID    Parts/Axis  ＜ICS_Right_Lower SIO7,SIO8＞
// [R06] 右股/ロール
// [R07] 右股/ピッチ
// [R08] 右膝/ピッチ
// [R09] 右足首/ピッチ
// [R10] 右足首/ロール

namespace meridian {
namespace modules {
namespace plugin {

class MrdPadKRC5FH : public IMeridianPad {
public:
  MrdPadKRC5FH() {}
  ~MrdPadKRC5FH() {}

public:
  const char *type_name() override { return "KRC-5FH"; };

public:
  bool setup() override { return true; };
  bool input(Meridim90 &a_meridim) override { return true; };
  bool output(Meridim90 &a_meridim) override { return true; };
};

}; // namespace plugin
}; // namespace modules
}; // namespace meridian

#endif // __MRD_MODULE_PAD_KRC5FH_HPP__
