/**
 * @file i_mrd_conversation.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-22
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_COMMUNICATION_HPP
#define I_MRD_COMMUNICATION_HPP

#include "Meridim90.hpp"

namespace meridian {
namespace core {
namespace communication {
using namespace meridian::core::meridim;

class IMeridianConversation {
public:
  virtual bool setup() = 0;

  virtual bool received(Meridim90 &a_meridim) = 0;
  virtual bool send(Meridim90 &a_meridim) = 0;

  virtual bool with_skip() = 0;
};

} // namespace communication
} // namespace core
} // namespace meridian

#endif // I_MRD_COMMUNICATION_HPP
