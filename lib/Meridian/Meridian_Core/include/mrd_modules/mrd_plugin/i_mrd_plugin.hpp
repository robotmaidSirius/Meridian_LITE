/**
 * @file i_mrd_plugin.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-20
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __I_MRD_PLUGIN_HPP__
#define __I_MRD_PLUGIN_HPP__

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
  virtual bool input(Meridim90 &a_meridim) = 0;
  virtual bool processing(Meridim90 &a_meridim) = 0;
  virtual bool output(Meridim90 &a_meridim) = 0;

public:
  void set_diagnostic(IMeridianDiagnostic &ref) { this->a_diag = &ref; }

protected:
  IMeridianDiagnostic *a_diag;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_HPP__
