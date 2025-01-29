/**
 * @file i_mrd_plugin_spi.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-24
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __I_MRD_PLUGIN_SPI_HPP__
#define __I_MRD_PLUGIN_SPI_HPP__

#include "i_mrd_plugin.hpp"

namespace meridian {
namespace modules {
namespace plugin {

class IMeridianSPIStatus {
public:
  bool initalized = false;
  bool setup = false;
  bool happened_error = false;

public:
  void all_ok() {
    this->initalized = true;
    this->setup = true;
    this->happened_error = false;
  }
};

class IMeridianSPI : public IMeridianPlugin {
public:
  void get_status(IMeridianSPIStatus &state) {
    state.initalized = this->a_state.initalized;
    state.setup = this->a_state.setup;
    state.happened_error = this->a_state.happened_error;
  }

protected:
  IMeridianSPIStatus a_state;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_SPI_HPP__
