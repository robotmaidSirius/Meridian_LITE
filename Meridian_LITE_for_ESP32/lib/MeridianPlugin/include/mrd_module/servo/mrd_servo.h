#ifndef __MERIDIAN_SERVO_DISTRIBUTOR_H__
#define __MERIDIAN_SERVO_DISTRIBUTOR_H__

// ライブラリ導入
#include <Meridim90.hpp> // Meridim90のライブラリ導入

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoNone {
public:
  bool begin() { return true; }
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_SERVO_DISTRIBUTOR_H__
