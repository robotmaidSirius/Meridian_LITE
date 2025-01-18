#ifndef __MERIDIAN_CONFIG__
#define __MERIDIAN_CONFIG__

//-------------------------------------------------------------------------
//  各種設定
//-------------------------------------------------------------------------

// Meridimの基本設定
#define FRAME_DURATION 10  // 1フレームあたりの単位時間（単位ms）
#define CHARGE_TIME    200 // 起動時のコンデンサチャージ待機時間（単位ms）

// 動作モード
#define MODE_ESP32_STANDALONE 0 // ESP32をボードに挿さず動作確認（0:NO, 1:YES）
#define MODE_UDP_RECEIVE      1 // PCからのデータ受信（0:OFF, 1:ON, 通常は1）
#define MODE_UDP_SEND         1 // PCへのデータ送信（0:OFF, 1:ON, 通常は1）

// Wifiの設定(SSID,パスワード等は別途keys.hで指定)
#define MODE_FIXED_IP 0 // IPアドレスを固定するか（0:NO, 1:YES）
#define UDP_TIMEOUT   4 // UDPの待受タイムアウト（単位ms,推奨値0）

// 動作チェックモード
#define CHECK_EEPROM_RW 0 // 起動時のEEPROMの動作チェック

// シリアルモニタリング
#define MONITOR_FRAME_DELAY       1    // シリアルモニタでフレーム遅延時間を表示（0:OFF, 1:ON）
#define MONITOR_FLOW              0    // シリアルモニタでフローを表示（0:OFF, 1:ON）
#define MONITOR_ERR_SERVO         0    // シリアルモニタでサーボエラーを表示（0:OFF, 1:ON）
#define MONITOR_ERR_ALL           0    // 全経路の受信エラー率を表示
#define MONITOR_SEQ               0    // シリアルモニタでシーケンス番号チェックを表示（0:OFF, 1:ON）
#define MONITOR_PAD               0    // シリアルモニタでリモコンのデータを表示（0:OFF, 1:ON）
#define MONITOR_SUPPRESS_DURATION 8000 // 起動直後のタイムアウトメッセージ抑制時間(単位ms)

// I2C設定, I2Cセンサ関連設定
#define I2C0_SPEED 400000 // I2Cの速度（400kHz推奨）
// #define I2C1_SPEED 100000  // I2Cの速度（100kHz推奨?）
// #define I2C1_MERIMOTE_ADDR 0x58 // MerimoteのI2Cアドレス

// SPI設定
#define SPI0_SPEED 6000000 // SPI通信の速度（6000000kHz推奨）

// PC接続関連設定
#define SERIAL_PC_BPS     115200 // PCとのシリアル速度（モニタリング表示用）
#define SERIAL_PC_TIMEOUT 2000   // PCとのシリアル接続確立タイムアウト(ms)

// JOYPAD関連設定
#define PAD_INIT_TIMEOUT 10000 // 起動時のJOYPADの接続確立のタイムアウト(ms)
#define PAD_INTERVAL     10    // JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)
#define PAD_BUTTON_MARGE 1     // 0:JOYPADのボタンデータをMeridim受信値に論理積, 1:Meridim受信値に論理和
#define PAD_GENERALIZE   1     // ジョイパッドの入力値をPS系に一般化する

// ピンアサイン
#define PIN_ERR_LED       25 // LED用 処理が時間内に収まっていない場合に点灯
#define PIN_EN_L          33 // サーボL系統のENピン
#define PIN_EN_R          4  // サーボR系統のENピン
#define PIN_CHIPSELECT_SD 15 // SDカード用のCSピン
#define PIN_I2C0_SDA      22 // I2CのSDAピン
#define PIN_I2C0_SCL      21 // I2CのSCLピン
#define PIN_LED_BT        26 // Bluetooth接続確認用ピン(点滅はペアリング,点灯でリンク確立)

#endif // __MERIDIAN_CONFIG__
