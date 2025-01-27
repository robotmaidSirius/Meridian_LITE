/**
 * @file mrd_module_pad_wiimote.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_PAD_WIIMOTE_HPP__
#define __MRD_MODULE_PAD_WIIMOTE_HPP__

// ヘッダーファイルの読み込み
#include "mrd_modules/mrd_plugin/i_mrd_plugin_pad.hpp"

// ライブラリ導入

namespace meridian {
namespace modules {
namespace plugin {

class MrdPadWiimote : public IMeridianPad {
public:
  MrdPadWiimote() {}
  ~MrdPadWiimote() {}

public:
  const char *type_name() override { return "Wiimote"; };

public:
  bool setup() override { return true; };
  bool input(Meridim90 &a_meridim) override { return true; };
  bool output(Meridim90 &a_meridim) override { return true; };
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_PAD_WIIMOTE_HPP__
