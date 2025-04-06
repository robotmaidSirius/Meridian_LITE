/**
 * @file meridian_board_lite.cpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#include "board/meridian_board_lite.hpp"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

namespace meridian {
namespace board {
namespace meridian_board_lite {
using namespace meridian::core::meridim;

Meridim90 meridim90;
mrd_entity *entity = nullptr;
mrd_parameters param;
mrd_diagnosis diagnosis;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

//////////////////////////////////////////////////////////////////////////
// 内部関数
//////////////////////////////////////////////////////////////////////////

// 起動メッセージの表示(バージョン, PC-USB,SPI0,i2c0のスピード)
void hello_meridian_board_lite() {
  bool output_log = true;
  if (nullptr == entity) {
    output_log = false;
  }
  if (output_log) {
    entity->communication.diag->log_info("Hi, This is %s(%s) %s.", PLUGIN_BOARD_NAME, PLUGIN_NAME, PLUGIN_VERSION);
    // Meridian Core
    entity->communication.diag->log_info("  Debug port: %s (%d bps)", entity->communication.diag->get_name(), BOARD_SETTING_DEFAULT_SERIAL0_BAUD);
    entity->communication.diag->log_info("  Communication: %s", (nullptr != entity->communication.con) ? entity->communication.con->get_name() : "--");

    // Meridian Plugin
    entity->communication.diag->log_info("  Mounted Plugin");
    for (int i = 0; i < MERIDIAN_BOARD_LITE_ANALOG_NUM; i++) {
      entity->communication.diag->log_info("    Analog[%d] : %s", i, (nullptr != entity->plugin.analog[i]) ? entity->plugin.analog[i]->get_name() : "--");
    }
    for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
      entity->communication.diag->log_info("    DAC[%d] : %s", i, (nullptr != entity->plugin.gpio[i]) ? entity->plugin.gpio[i]->get_name() : "--");
    }
    entity->communication.diag->log_info("    I2C: %u bps", param.i2c_speed);
    for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
      entity->communication.diag->log_info("      I2C[%d] : %s", i, (nullptr != entity->plugin.i2c[i]) ? entity->plugin.i2c[i]->get_name() : "--");
    }
    entity->communication.diag->log_info("    EEPROM : %s", (nullptr != entity->plugin.eeprom) ? entity->plugin.eeprom->get_name() : "--");
    entity->communication.diag->log_info("    SD-CARD : %s", (nullptr != entity->plugin.sd_card) ? entity->plugin.sd_card->get_name() : "--");
    entity->communication.diag->log_info("    SPI: %u bps", param.spi_speed);
    entity->communication.diag->log_info("      Common SPI : %s", (nullptr != entity->plugin.spi) ? entity->plugin.spi->get_name() : "--");
    entity->communication.diag->log_info("    ICS_L : %s (%u bps)", (nullptr != entity->plugin.servo_left) ? entity->plugin.servo_left->get_name() : "--", param.ics_l_speed);
    entity->communication.diag->log_info("    ICS_R : %s (%u bps)", (nullptr != entity->plugin.servo_right) ? entity->plugin.servo_right->get_name() : "--", param.ics_r_speed);
    entity->communication.diag->log_info("    Pad : %s ", (nullptr != entity->plugin.pad) ? entity->plugin.pad->get_name() : "--");
  }
}

/// @brief 指定された時間だけ待機する関数
void boot_standby(bool output_log = true) {
  if (nullptr == entity) {
    output_log = false;
  }
  if (output_log) {
    entity->communication.diag->log_info("Charging the capacitor.");
  }
  for (int i = 0; i < BOARD_SETTING_MOUNTED_BOOT_STANDBY; i++) {
    if (0 == (i % 100)) {
      // 100msごとにピリオドを表示
      if (output_log) {
        entity->communication.diag->log(".");
      }
    }
    delay(1);
  }
  if (output_log) {
    entity->communication.diag->log("\n");
  }
}

//////////////////////////////////////////////////////////////////////////
// 公開関数
//////////////////////////////////////////////////////////////////////////

bool board_setup(mrd_entity *a_entity, mrd_parameters *a_param) {
  bool result = true;
  //////////////////////////////////////////////////////////
  // Initialize the shared memory
  //////////////////////////////////////////////////////////
  if (nullptr != a_param) {
    param = *a_param;
  }
  entity = a_entity;
  //////////////////////////////////////////////////////////
  // Arduino Setup
  //////////////////////////////////////////////////////////
  // Setting Serial
  if (PINS_DEFAULT_SERIAL0_RX != -1 && PINS_DEFAULT_SERIAL0_TX != -1) {
    if (PINS_DEFAULT_SERIAL0_RX == SOC_RX0 && PINS_DEFAULT_SERIAL0_TX == SOC_TX0) {
      Serial.begin(BOARD_SETTING_DEFAULT_SERIAL0_BAUD, BOARD_SETTING_DEFAULT_SERIAL0_CONFIG);
    } else {
      Serial.begin(BOARD_SETTING_DEFAULT_SERIAL0_BAUD, BOARD_SETTING_DEFAULT_SERIAL0_CONFIG, PINS_DEFAULT_SERIAL0_RX, PINS_DEFAULT_SERIAL0_TX);
    }
  }
#if SOC_UART_NUM > 1
  if (PINS_DEFAULT_SERIAL1_RX != -1 && PINS_DEFAULT_SERIAL1_TX != -1) {
    if (PINS_DEFAULT_SERIAL1_RX == RX1 && PINS_DEFAULT_SERIAL1_TX == TX1) {
      Serial1.begin(BOARD_SETTING_DEFAULT_SERIAL1_BAUD, BOARD_SETTING_DEFAULT_SERIAL1_CONFIG);
    } else {
      Serial1.begin(BOARD_SETTING_DEFAULT_SERIAL1_BAUD, BOARD_SETTING_DEFAULT_SERIAL1_CONFIG, PINS_DEFAULT_SERIAL1_RX, PINS_DEFAULT_SERIAL1_TX);
    }
  }
#endif
#if SOC_UART_NUM > 2
  if (PINS_DEFAULT_SERIAL2_RX != -1 && PINS_DEFAULT_SERIAL2_TX != -1) {
    if (PINS_DEFAULT_SERIAL2_RX == RX2 && PINS_DEFAULT_SERIAL2_TX == TX2) {
      Serial2.begin(BOARD_SETTING_DEFAULT_SERIAL2_BAUD, BOARD_SETTING_DEFAULT_SERIAL2_CONFIG);
    } else {
      Serial2.begin(BOARD_SETTING_DEFAULT_SERIAL2_BAUD, BOARD_SETTING_DEFAULT_SERIAL2_CONFIG, PINS_DEFAULT_SERIAL2_RX, PINS_DEFAULT_SERIAL2_TX);
    }
  }
#endif
  if (nullptr != entity) {
    if (nullptr != entity->communication.diag) {
      result = entity->communication.diag->setup();
    } else {
      result = false;
    }
    if (false == result) {
      entity->communication.diag = new IMeridianDiagnostic();
      entity->communication.diag->setup();
      result = true;
    }
  }

  // Setting I2C
  bool flag_ic2_begin = false;
  if (nullptr != entity) {
    for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
      if (nullptr != entity->plugin.i2c[i]) {
        flag_ic2_begin = true;
        break;
      }
    }
  }
  if (flag_ic2_begin) {
    result = Wire.begin(PINS_DEFAULT_I2C_SDA, PINS_DEFAULT_I2C_SCL, param.i2c_speed);
    if (false == result) {
      entity->communication.diag->log_error("Failed to setup I2C");
    }
  }

  // Setting SPI
  SPI.setFrequency(param.spi_speed);

  //////////////////////////////////////////////////////////
  // Booting
  //////////////////////////////////////////////////////////
  // ボード搭載のコンデンサの充電時間として待機
  boot_standby();

  //////////////////////////////////////////////////////////
  // Setup Communication
  //////////////////////////////////////////////////////////
  if (result) {
    diagnosis.communication.con.initalized = false;
    if (nullptr != entity) {
      if (nullptr != entity->communication.con) {
        entity->communication.con->set_diagnostic(*entity->communication.diag);
        diagnosis.communication.con.initalized = entity->communication.con->setup();
        result &= diagnosis.communication.con.initalized;
      }
    }
    if (false == diagnosis.communication.con.initalized) {
      entity->communication.diag->log_fatal("Failed to Setup/communication::conversation");
    }

    //////////////////////////////////////////////////////////
    // Setup Plugin
    //////////////////////////////////////////////////////////
    if (nullptr != entity) {
      for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
        if (nullptr != entity->plugin.gpio[i]) {
          entity->plugin.gpio[i]->set_diagnostic(*entity->communication.diag);
          diagnosis.plugin.gpio[i].initalized = entity->plugin.gpio[i]->setup();
          result &= diagnosis.plugin.gpio[i].initalized;
          if (false == diagnosis.plugin.gpio[i].initalized) {
            entity->communication.diag->log_error("Failed to Setup/plugin::gpio[%d]", i);
          }
        } else {
          diagnosis.plugin.gpio[i].all_ok();
        }
      }
      for (int i = 0; i < MERIDIAN_BOARD_LITE_ANALOG_NUM; i++) {
        if (nullptr != entity->plugin.analog[i]) {
          entity->plugin.analog[i]->set_diagnostic(*entity->communication.diag);
          diagnosis.plugin.analog[i].initalized = entity->plugin.analog[i]->setup();
          result &= diagnosis.plugin.analog[i].initalized;
          if (false == diagnosis.plugin.analog[i].initalized) {
            entity->communication.diag->log_error("Failed to Setup/plugin::analog[%d]", i);
          }
        } else {
          diagnosis.plugin.analog[i].all_ok();
        }
      }
      for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
        if (nullptr != entity->plugin.i2c[i]) {
          entity->plugin.i2c[i]->set_diagnostic(*entity->communication.diag);
          diagnosis.plugin.i2c[i].initalized = entity->plugin.i2c[i]->setup();
          result &= diagnosis.plugin.i2c[i].initalized;
          if (false == diagnosis.plugin.i2c[i].initalized) {
            entity->communication.diag->log_error("Failed to Setup/plugin::i2c[%d]", i);
          }
        } else {
          diagnosis.plugin.i2c[i].all_ok();
        }
      }
      if (nullptr != entity->plugin.eeprom) {
        entity->plugin.eeprom->set_diagnostic(*entity->communication.diag);
        diagnosis.plugin.eeprom.initalized = entity->plugin.eeprom->setup();
        result &= diagnosis.plugin.eeprom.initalized;
        if (false == diagnosis.plugin.eeprom.initalized) {
          entity->communication.diag->log_error("Failed to Setup/plugin::eeprom");
        }
      } else {
        diagnosis.plugin.eeprom.all_ok();
      }
      if (nullptr != entity->plugin.sd_card) {
        entity->plugin.sd_card->set_diagnostic(*entity->communication.diag);
        diagnosis.plugin.sd_card.initalized = entity->plugin.sd_card->setup();
        result &= diagnosis.plugin.sd_card.initalized;
        if (false == diagnosis.plugin.sd_card.initalized) {
          entity->communication.diag->log_error("Failed to Setup/plugin::sd_card");
        }
      } else {
        diagnosis.plugin.sd_card.all_ok();
      }
      if (nullptr != entity->plugin.spi) {
        entity->plugin.spi->set_diagnostic(*entity->communication.diag);
        diagnosis.plugin.spi.initalized = entity->plugin.spi->setup();
        result &= diagnosis.plugin.spi.initalized;
        if (false == diagnosis.plugin.spi.initalized) {
          entity->communication.diag->log_error("Failed to Setup/plugin::spi");
        }
      } else {
        diagnosis.plugin.spi.all_ok();
      }
      if (nullptr != entity->plugin.servo_left) {
        entity->plugin.servo_left->set_diagnostic(*entity->communication.diag);
        diagnosis.plugin.servo_left.initalized = entity->plugin.servo_left->setup();
        result &= diagnosis.plugin.servo_left.initalized;
        if (false == diagnosis.plugin.servo_left.initalized) {
          entity->communication.diag->log_error("Failed to Setup/plugin::servo_left");
        }
      } else {
        diagnosis.plugin.servo_left.all_ok();
      }
      if (nullptr != entity->plugin.servo_right) {
        entity->plugin.servo_right->set_diagnostic(*entity->communication.diag);
        diagnosis.plugin.servo_right.initalized = entity->plugin.servo_right->setup();
        result &= diagnosis.plugin.servo_right.initalized;
        if (false == diagnosis.plugin.servo_right.initalized) {
          entity->communication.diag->log_error("Failed to Setup/plugin::servo_right");
        }
      } else {
        diagnosis.plugin.servo_right.all_ok();
      }
      if (nullptr != entity->plugin.pad) {
        entity->plugin.pad->set_diagnostic(*entity->communication.diag);
        diagnosis.plugin.pad.initalized = entity->plugin.pad->setup();
        result &= diagnosis.plugin.pad.initalized;
        if (false == diagnosis.plugin.pad.initalized) {
          entity->communication.diag->log_error("Failed to Setup/plugin::pad");
        }
      } else {
        diagnosis.plugin.pad.all_ok();
      }
    }
  }
  if (result) {
    meridian::core::execution::meridim_clear(meridim90);
    hello_meridian_board_lite();
    mrd_timer_setup(BOARD_SETTING_DEFAULT_INTERVAL_MS); // タイマーの設定
  } else {
    entity->communication.diag->log_fatal("Failed setup");
  }
  return result;
}

Meridim90 mrd_input() {
  static Meridim90 a_meridim90;
  //////////////////////////////////////////////////////////
  // Copy the data from the shared memory
  //////////////////////////////////////////////////////////
  int result_lock = pthread_mutex_lock(&mutex);
  if (0 == result_lock) {
    memcpy(&a_meridim90, &meridim90, sizeof(Meridim90));
  }
  result_lock = pthread_mutex_unlock(&mutex);
  if (0 != result_lock) {
    log_e("[%d] %s", result_lock, "can not unlock");
  }
  bool result = true;
  if (nullptr != entity) {
    //////////////////////////////////////////////////////////
    // Input Communication
    //////////////////////////////////////////////////////////
    if (nullptr != entity->communication.con) {
      // TODO: 通信処理は失敗してもOK
      entity->communication.con->notify_received(a_meridim90);
    }
    //////////////////////////////////////////////////////////
    // Input Plugin
    //////////////////////////////////////////////////////////
    for (int i = 0; i < MERIDIAN_BOARD_LITE_ANALOG_NUM; i++) {
      if (nullptr != entity->plugin.analog[i]) {
        result &= entity->plugin.analog[i]->input(a_meridim90);
      }
    }
    for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
      if (nullptr != entity->plugin.gpio[i]) {
        result &= entity->plugin.gpio[i]->input(a_meridim90);
      }
    }
    for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
      if (nullptr != entity->plugin.i2c[i]) {
        result &= entity->plugin.i2c[i]->input(a_meridim90);
      }
    }
    if (nullptr != entity->plugin.servo_left) {
      result &= entity->plugin.servo_left->input(a_meridim90);
    }
    if (nullptr != entity->plugin.servo_right) {
      result &= entity->plugin.servo_right->input(a_meridim90);
    }
    if (nullptr != entity->plugin.spi) {
      result &= entity->plugin.spi->input(a_meridim90);
    }
    if (nullptr != entity->plugin.eeprom) {
      result &= entity->plugin.eeprom->input(a_meridim90);
    }
    if (nullptr != entity->plugin.sd_card) {
      result &= entity->plugin.sd_card->input(a_meridim90);
    }
    if (nullptr != entity->plugin.pad) {
      result &= entity->plugin.pad->input(a_meridim90);
    }
  }
  return a_meridim90;
}
bool mrd_output(Meridim90 &a_meridim90) {
  bool result = true;
  //////////////////////////////////////////////////////////
  // Output Plugin
  //////////////////////////////////////////////////////////
  for (int i = 0; i < MERIDIAN_BOARD_LITE_ANALOG_NUM; i++) {
    if (nullptr != entity->plugin.analog[i]) {
      result &= entity->plugin.analog[i]->output(a_meridim90);
    }
  }
  for (int i = 0; i < MERIDIAN_BOARD_LITE_GPIO_NUM; i++) {
    if (nullptr != entity->plugin.gpio[i]) {
      result &= entity->plugin.gpio[i]->output(a_meridim90);
    }
  }
  for (int i = 0; i < MERIDIAN_BOARD_LITE_I2C_NUM; i++) {
    if (nullptr != entity->plugin.i2c[i]) {
      result &= entity->plugin.i2c[i]->output(a_meridim90);
    }
  }
  if (nullptr != entity->plugin.servo_left) {
    result &= entity->plugin.servo_left->output(a_meridim90);
  }
  if (nullptr != entity->plugin.servo_right) {
    result &= entity->plugin.servo_right->output(a_meridim90);
  }
  if (nullptr != entity->plugin.spi) {
    result &= entity->plugin.spi->output(a_meridim90);
  }
  if (nullptr != entity->plugin.sd_card) {
    result &= entity->plugin.sd_card->output(a_meridim90);
  }
  if (nullptr != entity->plugin.eeprom) {
    result &= entity->plugin.eeprom->output(a_meridim90);
  }
  if (nullptr != entity->plugin.pad) {
    result &= entity->plugin.pad->output(a_meridim90);
  }
  if (result) {
    //////////////////////////////////////////////////////////
    // Output Communication
    //////////////////////////////////////////////////////////
    if (nullptr != entity->communication.con) {
      entity->communication.con->notify_send(a_meridim90);
    }
    //////////////////////////////////////////////////////////
    // Copy the data to the shared memory
    //////////////////////////////////////////////////////////
    int result_lock = pthread_mutex_lock(&mutex);
    if (0 == result_lock) {
      memcpy(&meridim90, &a_meridim90, sizeof(Meridim90));
    }
    result_lock = pthread_mutex_unlock(&mutex);
    if (0 != result_lock) {
      log_e("[%d] %s", result_lock, "can not unlock");
    }
  }
  return result;
}

} // namespace meridian_board_lite
} // namespace board
} // namespace meridian
