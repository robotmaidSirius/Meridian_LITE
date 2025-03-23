/**
 * @file app_sd.cpp
 * @brief SD-Cardの関数群
 * @version 1.2.0
 * @date 2025-03-03
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "application/mrd_app.hpp"
#include "mrd_module/filesystem/mrd_sd.h"

meridian::modules::plugin::MrdFsSdCard mrd_sd(PIN_CHIPSELECT_SD);

void app_sd_setup() {
  if (SD_MOUNT) {
    Serial.print("Initializing SD-Card... ");
    if (mrd_sd.init()) {
      Serial.println("OK.");
      if (SD_CHECK_RW) {
        mrd_sd.check();
      }
    } else {
      Serial.println("Card failed, or not present.");
    }
  } else {
    Serial.println("SD NOT mounted.");
  }
}
