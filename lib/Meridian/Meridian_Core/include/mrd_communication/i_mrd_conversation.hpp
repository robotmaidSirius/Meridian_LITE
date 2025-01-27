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

class IMeridianConversation {
public:
  virtual const char *type_name() { return "None"; };
  virtual bool setup() = 0;

  virtual bool received(Meridim90 &a_meridim) = 0;
  virtual bool send(Meridim90 &a_meridim) = 0;

public:
  void set_diagnostic(IMeridianDiagnostic &ref) { this->a_diag = &ref; }

protected:
  IMeridianDiagnostic *a_diag;
};

} // namespace communication
} // namespace core
} // namespace meridian

#endif // __I_MRD_COMMUNICATION_HPP__
