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
  virtual const char *get_name() { return "Unknow"; }
  virtual bool setup() = 0;
  virtual bool input(Meridim90 &a_meridim) = 0;
  virtual bool output(Meridim90 &a_meridim) = 0;

public:
  class Status {
  public:
    static const int OPTION_MAX = 10;

    bool initalized = false;
    bool setup = false;
    bool happened_error = false;
    bool option[Status::OPTION_MAX] = {false};

  public:
    void all_ok() {
      this->initalized = true;
      this->setup = true;
      this->happened_error = false;
      for (int i = 0; i < Status::OPTION_MAX; i++) {
        this->option[i] = false;
      }
    }
  };

public:
  virtual void set_diagnostic(IMeridianDiagnostic &ref) { this->m_diag = &ref; }
  void get_status(Status &state) {
    state.initalized = this->m_state.initalized;
    state.setup = this->m_state.setup;
    state.happened_error = this->m_state.happened_error;
    for (int i = 0; i < Status::OPTION_MAX; i++) {
      state.option[i] = this->m_state.option[i];
    }
  }

protected:
  IMeridianDiagnostic *m_diag;
  Status m_state;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_HPP__
