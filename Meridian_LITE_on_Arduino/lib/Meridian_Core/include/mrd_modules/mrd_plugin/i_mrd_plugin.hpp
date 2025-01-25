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
#include "mrd_communication/i_mrd_diagnostic.hpp"

namespace meridian {
namespace modules {
namespace plugin {
using namespace meridian::core::meridim;
using namespace meridian::core::communication;

class IMeridianPlugin {
public:
  virtual bool setup() = 0;
  virtual bool refresh(Meridim90 &a_meridim) = 0;

public:
  void set_diagnostic(IMeridianDiagnostic &ref) { this->a_diag = &ref; }

protected:
  IMeridianDiagnostic *a_diag;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // I_MRD_PLUGIN_HPP
