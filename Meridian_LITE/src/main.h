#ifndef __MERIDIAN_MAIN_FUNC__
#define __MERIDIAN_MAIN_FUNC__

// ヘッダファイルの読み込み

//------------------------------------------------------------------------------------
//  列挙型
//------------------------------------------------------------------------------------

enum BinHexDec { // 数値表示タイプの列挙型(Bin, Hex, Dec)
  Bin = 0,       // BIN
  Hex = 1,       // HEX
  Dec = 2,       // DEC
};

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

// フラグ用変数
struct MrdFlags {
  bool imuahrs_available = true;            // メインセンサ値を読み取る間, サブスレッドによる書き込みを待機
  bool udp_board_passive = false;           // UDP通信の周期制御がボード主導(false) か, PC主導(true)か.
  bool count_frame_reset = false;           // フレーム管理時計をリセットする
  bool stop_board_during = false;           // ボードの末端処理をmeridim[2]秒, meridim[3]ミリ秒だけ止める.
  bool eeprom_write_mode = false;           // EEPROMへの書き込みモード.
  bool eeprom_read_mode = false;            // EEPROMからの読み込みモード.
  bool eeprom_protect = EEPROM_PROTECT;     // EEPROMの書き込みプロテクト.
  bool eeprom_load = EEPROM_LOAD;           // 起動時にEEPROMの内容を読み込む
  bool eeprom_set = EEPROM_SET;             // 起動時にEEPROMに規定値をセット
  bool sdcard_write_mode = false;           // SDCARDへの書き込みモード.
  bool sdcard_read_mode = false;            // SDCARDからの読み込みモード.
  bool wire0_init = false;                  // I2C 0系統の初期化合否
  bool wire1_init = false;                  // I2C 1系統の初期化合否
  bool bt_busy = false;                     // Bluetoothの受信中フラグ（UDPコンフリクト回避用）
  bool spi_rcvd = true;                     // SPIのデータ受信判定
  bool udp_rcvd = false;                    // UDPのデータ受信判定
  bool udp_busy = false;                    // UDPスレッドでの受信中フラグ（送信抑制）
  bool udp_receive_mode = MODE_UDP_RECEIVE; // PCからのデータ受信実施（0:OFF, 1:ON, 通常は1）
  bool udp_send_mode = MODE_UDP_SEND;       // PCへのデータ送信実施（0:OFF, 1:ON, 通常は1）
  bool meridim_rcvd = false;                // Meridimが正しく受信できたか.
};
MrdFlags flg;

// サーボ用変数
struct ServoParam {
  // サーボの最大接続 (サーボ送受信のループ処理数）
  int num_max;

  // 各サーボのマウントありなし(config.hで設定)
  int ixl_mount[IXL_MAX]; // L系統
  int ixr_mount[IXR_MAX]; // R系統

  // 各サーボのコード上のインデックスに対し, 実際に呼び出すハードウェアのID番号(config.hで設定)
  int ixl_id[IXL_MAX]; // L系統の実サーボ呼び出しID番号
  int ixr_id[IXR_MAX]; // R系統の実サーボ呼び出しID番号

  // 各サーボの正逆方向補正用配列(config.hで設定)
  int ixl_cw[IXL_MAX]; // L系統
  int ixr_cw[IXR_MAX]; // R系統

  // 各サーボの直立ポーズトリム値(config.hで設定)
  float ixl_trim[IXL_MAX]; // L系統
  float ixr_trim[IXR_MAX]; // R系統

  // 各サーボのポジション値(degree)
  float ixl_tgt[IXL_MAX] = {0};      // L系統の目標値
  float ixr_tgt[IXR_MAX] = {0};      // R系統の目標値
  float ixl_tgt_past[IXL_MAX] = {0}; // L系統の前回の値
  float ixr_tgt_past[IXR_MAX] = {0}; // R系統の前回の値

  // サーボのエラーカウンタ配列
  int ixl_err[IXL_MAX] = {0}; // L系統
  int ixr_err[IXR_MAX] = {0}; // R系統

  // サーボのコンディションステータス配列
  uint16_t ixl_stat[IXL_MAX] = {0}; // L系統サーボのコンディションステータス配列
  uint16_t ixr_stat[IXR_MAX] = {0}; // R系統サーボのコンディションステータス配列
};
ServoParam sv;

//================================================================================================================
//  関数各種
//================================================================================================================

#endif //__MERIDIAN_MAIN_FUNC__
