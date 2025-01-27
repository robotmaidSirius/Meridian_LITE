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
#include <mrd_modules/mrd_plugin/i_mrd_plugin_pad.hpp>

// ライブラリ導入

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
