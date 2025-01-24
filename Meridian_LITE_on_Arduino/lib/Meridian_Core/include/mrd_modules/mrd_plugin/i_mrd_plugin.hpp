/**
 * @file i_mrd_plugin.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-20
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_PLUGIN_HPP
#define I_MRD_PLUGIN_HPP

#include "Meridim90.hpp"

namespace meridian {
namespace modules {
namespace plugin {
using namespace meridian::core::meridim;

class I_Meridian_Plugin {
public:
  virtual bool setup() = 0;
  virtual bool refresh(Meridim90 &a_meridim) = 0;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // I_MRD_PLUGIN_HPP
