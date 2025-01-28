/**
 * @file i_mrd_conversation.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-22
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __I_MRD_COMMUNICATION_HPP__
#define __I_MRD_COMMUNICATION_HPP__

#include "Meridim90.hpp"
#include "i_mrd_diagnostic.hpp"

namespace meridian {
namespace core {
namespace communication {
using namespace meridian::core::meridim;

class IMeridianConversationStatus {
public:
  bool initalized = false;
  bool setup = false;
  bool happened_error = false;
  bool no_device = false;

public:
  void all_ok() {
    this->initalized = true;
    this->setup = true;
    this->happened_error = false;
    this->no_device = true;
  }
};

class IMeridianConversation {
public:
  virtual const char *type_name() { return "None"; };
  virtual bool setup() = 0;

  virtual bool received(Meridim90 &a_meridim) = 0;
  virtual bool send(Meridim90 &a_meridim) = 0;

public:
  virtual void set_diagnostic(IMeridianDiagnostic &ref) { this->a_diag = &ref; }
  void get_status(IMeridianConversationStatus &state) {
    state.initalized = this->a_state.initalized;
    state.setup = this->a_state.setup;
    state.happened_error = this->a_state.happened_error;
    state.no_device = this->a_state.no_device;
  }

protected:
  IMeridianDiagnostic *a_diag;
  IMeridianConversationStatus a_state;
};

} // namespace communication
} // namespace core
} // namespace meridian

#endif // __I_MRD_COMMUNICATION_HPP__
