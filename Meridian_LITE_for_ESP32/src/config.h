#ifndef __MERIDIAN_CONFIG__
#define __MERIDIAN_CONFIG__

#include <mrd_module/imu/mrd_module_imu.hpp>
#include <mrd_module/sv_common.hpp>

namespace meridian {
namespace core {
namespace execution {

} // namespace execution
} // namespace core
} // namespace meridian

namespace meridian {
namespace modules {
namespace config {

} // namespace config
} // namespace modules
} // namespace meridian

//==================================================================================================
//  MERIDIAN - LITE - ESP32の配線
//==================================================================================================
//
// ESP32devkitC  -  デバイス
//   3V3         -  BNO005 VIN
//   21          -  BNO005 SCL
//   22          -  BNO005 SDA
//   GND         -  BNO005 GND
//
//   4  EN       -  ICS変換基板 R系統 EN
//   16 RX       -  ICS変換基板 R系統 TX
//   17 TX       -  ICS変換基板 R系統 RX
//   5V          -  ICS変換基板 IOREF
//   GND         -  ICS変換基板 GND
//
//   33 EN       -  ICS変換基板 L系統 EN
//   32 RX       -  ICS変換基板 L系統 TX
//   27 TX       -  ICS変換基板 L系統 RX
//   5V          -  ICS変換基板 IOREF
//   GND         -  ICS変換基板 GND
//
//   27          -  SPI_MISO
//   23          -  SPI_MOSI
//   18          -  SPI_CSK
//   15          -  SPI_CS SD

//-------------------------------------------------------------------------
// ピンアサイン
//-------------------------------------------------------------------------
#define PIN_ERR_LED       25 // LED用 処理が時間内に収まっていない場合に点灯
#define PIN_EN_L          33 // サーボL系統のENピン
#define PIN_EN_R          4  // サーボR系統のENピン
#define PIN_CHIPSELECT_SD 15 // SDカード用のCSピン
#define PIN_I2C0_SDA      22 // I2CのSDAピン
#define PIN_I2C0_SCL      21 // I2CのSCLピン
#define PIN_LED_BT        26 // Bluetooth接続確認用ピン(点滅はペアリング,点灯でリンク確立)

//-------------------------------------------------------------------------
//  各種設定
//-------------------------------------------------------------------------
// Meridimの基本設定
#ifndef FRAME_DURATION
#define FRAME_DURATION 10 // 1フレームあたりの単位時間（単位ms）
#endif
#define CHARGE_TIME_MS 200 // 起動時のコンデンサチャージ待機時間（単位ms）

// 動作モード
#define MODE_ESP32_STANDALONE 0 // ESP32をボードに挿さず動作確認（0:NO, 1:YES）

//-------------------------------------------------------------------------
// Network
//-------------------------------------------------------------------------
// Wifiの設定(SSID,パスワード等は別途keys.hで指定)
#define NETWORK_UDP_RECEIVE 1 // PCからのデータ受信（0:OFF, 1:ON, 通常は1）
#define NETWORK_UDP_SEND    1 // PCへのデータ送信（0:OFF, 1:ON, 通常は1）
#define NETWORK_UDP_TIMEOUT 4 // UDPの待受タイムアウト（単位ms,推奨値0）

//-------------------------------------------------------------------------
// PC接続関連設定
//-------------------------------------------------------------------------
#define SERIAL_PC_BPS     115200 // PCとのシリアル速度（モニタリング表示用）
#define SERIAL_PC_TIMEOUT 2000   // PCとのシリアル接続確立タイムアウト(ms)

//-------------------------------------------------------------------------
// EEPROMの設定
//-------------------------------------------------------------------------
// 使用を確認できなかったのでコメントアウト
// #define EEPROM_SET      0   // 起動時にEEPROMにconfig.hの内容をセット(mrd_set_eeprom)
#define EEPROM_CHECK_RW 0   // 起動時のEEPROMの動作チェック
#define EEPROM_SIZE     100 // EEPROMでデータサイズ(2byte/1データ)
#define EEPROM_PROTECT  1   // EEPROMの書き込み保護(0:保護しない, 1:書き込み禁止)
#define EEPROM_LOAD     0   // 起動時にEEPROMの内容を諸設定にロードする
#define EEPROM_DUMP     0   // 起動時のEEPROM内容のダンプ表示

#define EEPROM_STYLE meridian::modules::plugin::MrdFsEEPROM::Hexadecimal::Hex // 起動時のEEPROM内容のダンプ表示の書式(Bin,Hex,Dec)

//-------------------------------------------------------------------------
// SD Cardの設定
//-------------------------------------------------------------------------
#define SD_MOUNT    0       // SDカードリーダーの有無 (0:なし, 1:あり)
#define SD_CHECK_RW 0       // 起動時のSDカードリーダーの読み書きチェック
#define SPI0_SPEED  6000000 // SPI通信の速度（6000000kHz推奨）

//-------------------------------------------------------------------------
// シリアルモニタリング
//-------------------------------------------------------------------------
#define MONITOR_FRAME_DELAY       1    // シリアルモニタでフレーム遅延時間を表示（0:OFF, 1:ON）
#define MONITOR_FLOW              0    // シリアルモニタでフローを表示（0:OFF, 1:ON）
#define MONITOR_ERR_SERVO         0    // シリアルモニタでサーボエラーを表示（0:OFF, 1:ON）
#define MONITOR_ERR_ALL           0    // 全経路の受信エラー率を表示
#define MONITOR_SEQ               0    // シリアルモニタでシーケンス番号チェックを表示（0:OFF, 1:ON）
#define MONITOR_PAD               0    // シリアルモニタでリモコンのデータを表示（0:OFF, 1:ON）
#define MONITOR_NETWORK           0    // シリアルモニタでネットワークの状態を表示（0:OFF, 1:ON）
#define MONITOR_SUPPRESS_DURATION 8000 // 起動直後のタイムアウトメッセージ抑制時間(単位ms)

//-------------------------------------------------------------------------
// IMU/AHRSの設定
// TODO: 動作確認をする
//-------------------------------------------------------------------------
// Defineで、MODULE_IMU_BNO055,MODULE_IMU_MPU6050を指定することで定義できるように変更
// #define IMUAHRS_MOUNT      meridian::modules::plugin::ImuAhrsType::NO_IMU // IMU/AHRSの搭載 NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS
#define IMUAHRS_I2C0_SPEED 400000 // I2Cの速度（400kHz推奨）
#define IMUAHRS_INTERVAL   10     // IMU/AHRSのセンサの読み取り間隔(ms)
#define IMUAHRS_STOCK      4      // MPUで移動平均を取る際の元にする時系列データの個数
// #define I2C1_SPEED 100000  // I2Cの速度（100kHz推奨?）
// #define I2C1_MERIMOTE_ADDR 0x58 // MerimoteのI2Cアドレス

//-------------------------------------------------------------------------
// JOYPAD関連設定
// TODO: 動作確認をする
//-------------------------------------------------------------------------
#define PAD_MOUNT        PC    // ジョイパッドの搭載 PC, MERIMOTE, BLUERETRO, KRR5FH, WIIMOTE
#define PAD_INIT_TIMEOUT 10000 // 起動時のJOYPADの接続確立のタイムアウト(ms)
#define PAD_INTERVAL     10    // JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)
#define PAD_BUTTON_MARGE 1     // 0:JOYPADのボタンデータをMeridim受信値に論理積, 1:Meridim受信値に論理和
#define PAD_GENERALIZE   1     // ジョイパッドの入力値をPS系に一般化する

//-------------------------------------------------------------------------
// サーボ設定
//-------------------------------------------------------------------------

// コマンドサーボの種類
// 00: NOSERVO (マウントなし),            01: PWM_S1 (Single PWM)[WIP]
// 11: PCA9685 (I2C_PCA9685toPWM)[WIP], 21: FTBRSX (FUTABA_RSxTTL)[WIP]
// 31: DXL1 (DYNAMIXEL 1.0)[WIP],       32: DXL2 (DYNAMIXEL 2.0)[WIP]
// 43: KOICS3 (KONDO_ICS 3.5 / 3.6),    44: KOPMX (KONDO_PMX)[WIP]
// 51: JRXBUS (JRPROPO_XBUS)[WIP]
// 61: FTCSTS (FEETECH_STS)[WIP],       62: FTCSCS (FEETECH_SCS)[WIP]
#define SERVO_MOUNT_TYPE_L KOICS3 // L系統のコマンドサーボの種類
#define SERVO_MOUNT_TYPE_R KOICS3 // R系統のコマンドサーボの種類

// 115200/625000/1250000 bps
#define SERVO_BAUDRATE_L 1250000 // L系統のICSサーボの通信速度bps
#define SERVO_BAUDRATE_R 1250000 // R系統のICSサーボの通信速度bps

#define SERVO_TIMEOUT_L     2 // L系統のICS返信待ちのタイムアウト時間
#define SERVO_TIMEOUT_R     2 // R系統のICS返信待ちのタイムアウト時間
#define SERVO_LOST_ERR_WAIT 6 // 連続何フレームサーボ信号をロストしたら異常とするか

#endif // __MERIDIAN_CONFIG__
