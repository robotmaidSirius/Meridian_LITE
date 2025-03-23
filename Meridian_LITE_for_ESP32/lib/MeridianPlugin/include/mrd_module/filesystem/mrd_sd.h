#ifndef __MERIDIAN_SD_H__
#define __MERIDIAN_SD_H__

// ライブラリ導入
#include <SD.h> // SDカード用

namespace meridian {
namespace modules {
namespace plugin {

class MrdFsSdCard {
private:
  int _chipselect_pin = -1; // SDカードのチップ選択ピン番号
  bool _initialed = false;  // 初期化済みフラグ

public:
  MrdFsSdCard(int a_chipselect_pin) {
    this->_chipselect_pin = a_chipselect_pin;
    this->_initialed = false;
  }

public:
  //------------------------------------------------------------------------------------
  //  初期化処理
  //------------------------------------------------------------------------------------

  /// @brief SDカードの初期化を試みる. SDカードがマウントされているか,
  ///        及びチップ選択ピンの設定に基づく.
  /// @return SDカードの初期化が成功した場合はtrueを,
  ///         失敗またはSDカードがマウントされていない場合はfalseを返す.
  bool init() {
    this->_initialed = false;
    if (0 <= this->_chipselect_pin) {
      if (SD.begin(this->_chipselect_pin)) {
        this->_initialed = true;
      }
    }
    return this->_initialed;
  }

  //------------------------------------------------------------------------------------
  //  リードライトテスト
  //------------------------------------------------------------------------------------

  /// @brief SDカードの読み書き機能をテストする. SDカードがマウントされ,
  /// 読み書きのチェックが要求された場合のみテストを実行する.
  /// @return SDカードの読み書きが成功した場合はtrueを, 失敗した場合はfalseを返す.
  bool check() {
    bool result = false;
    if (this->_initialed) {
      File sd_file; // SDカード用
      sd_file = SD.open("/test.txt", FILE_WRITE);
      delay(1); // SPI安定化検証用

      if (sd_file) {
        Serial.print("Checking SD card r/w... ");
        // SD書き込みテスト用のランダムな4桁の数字を生成
        randomSeed(long(analogRead(A0))); // 未接続ピンのノイズを利用
        int rand_number_tmp = random(1000, 9999);

        Serial.print("write code ");
        Serial.print(rand_number_tmp);
        // ファイルへの書き込みを実行
        sd_file.println(rand_number_tmp);
        delayMicroseconds(1); // SPI安定化検証用
        sd_file.close();
        delayMicroseconds(10); // SPI安定化検証用
        // ファイルからの読み込みを実行
        sd_file = SD.open("/test.txt");
        if (sd_file) {
          Serial.print(" and read code ");
          while (sd_file.available()) {
            Serial.write(sd_file.read());
          }
          sd_file.close();
        }
        SD.remove("/test.txt");
        delay(10);
        result = true;
      } else {
        Serial.println("Could not open SD test.txt file.");
        result = false;
      }
    }
    return result;
  }
};

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_SD_H__
