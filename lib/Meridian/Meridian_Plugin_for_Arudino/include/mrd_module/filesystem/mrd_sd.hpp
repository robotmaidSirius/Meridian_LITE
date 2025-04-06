/**
 * @file mrd_module_sd.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_MODULE_SD_HPP__
#define __MRD_MODULE_MODULE_SD_HPP__

// ヘッダーファイルの読み込み
#include <mrd_module/mrd_plugin/i_mrd_plugin_sd.hpp>

// ライブラリ導入
#include <SD.h> // SDカード用

#define MOUNT_SD          1  // SDカードリーダーの有無 (0:なし, 1:あり)
#define PIN_CHIPSELECT_SD 15 // SDカード用のCSピン
#define CHECK_SD_RW       1  // 起動時のSDカードリーダーの読み書きチェック

namespace meridian {
namespace modules {
namespace plugin {

namespace sd_card {

} // namespace sd_card

class MrdSdCard : public IMeridianSD {
private:
  bool mount_sd = false;
  int _chipselect_pin = -1;
  int _test_pin = A0;
  const char *_test_file = "/test.txt";

public:
  MrdSdCard(int chipselect_pin) { this->_chipselect_pin = chipselect_pin; }
  ~MrdSdCard() {}

public:
  bool input(Meridim90 &a_meridim) override { return true; }
  bool output(Meridim90 &a_meridim) override { return true; }
  bool write(uint16_t address, uint8_t data) override { return true; }
  uint8_t read(uint16_t address) override { return 0; }
  bool setup() override {
    if (this->mount_sd) {
      if (!SD.begin(this->_chipselect_pin)) {
        return false;
      } else {
        if (1 == CHECK_SD_RW) {
          return this->sd_check();
        }
      }
    }
    return true;
  }

  //------------------------------------------------------------------------------------
  //  リードライトテスト
  //------------------------------------------------------------------------------------

  /// @brief SDカードの読み書き機能をテストする. SDカードがマウントされ,
  /// 読み書きのチェックが要求された場合のみテストを実行する.
  /// @return SDカードの読み書きが成功した場合はtrueを, 失敗した場合はfalseを返す.
  bool sd_check() {
    bool result = false;
    if ((true == this->mount_sd)) {
      bool flag_write = false;
      File sd_file;                             // SDカード用
      uint8_t rand_number = random(0x10, 0xFF); // SD書き込みテスト用のランダムな4桁の数字を生成
      sd_file = SD.open(this->_test_file, FILE_WRITE);
      if (sd_file) {
        delay(1);                                      // SPI安定化検証用
        randomSeed(long(analogRead(this->_test_pin))); // 未接続ピンのノイズを利用
        {
          // ファイルへの書き込みを実行
          sd_file.println(rand_number);
          delayMicroseconds(1); // SPI安定化検証用
          sd_file.close();
          flag_write = true;
          delayMicroseconds(10); // SPI安定化検証用
        }
      }
      if (flag_write) {
        uint8_t rand_number_tmp;
        // ファイルからの読み込みを実行
        sd_file = SD.open(this->_test_file, FILE_READ);
        if (sd_file) {
          rand_number_tmp = sd_file.read();

          sd_file.close();
          if (rand_number == rand_number_tmp) {
            result = true;
          }
        }
      }
      if (flag_write) {
        result &= SD.remove(this->_test_file);
      }
    } else {
      result = true;
    }

    return result;
  }
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_MODULE_SD_HPP__
