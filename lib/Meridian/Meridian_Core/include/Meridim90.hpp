/**
 * @file Meridim90.hpp
 * @brief Meridianで定義されているMeridim90型
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MERIDIM90_HPP
#define MERIDIM90_HPP

#include <stdint.h>

//================================================================================================================
//  Meridim90配列 一覧表
//================================================================================================================
//
// [00]      マスターコマンド デフォルトは90 で配列数も同時に示す
// [01]      シーケンス番号
// [02]-[04] IMU/AHRS:acc＿x,acc＿y,acc＿z    加速度x,y,z
// [05]-[07] IMU/AHRS:gyro＿x,gyro＿y,gyro＿z ジャイロx,y,z
// [08]-[10] IMU/AHRS:mag＿x,mag＿y,mag＿z    磁気コンパスx,y,z
// [11]      IMU/AHRS:temp                   温度
// [12]-[14] IMU/AHRS:DMP ROLL,PITCH,YAW     DMP推定値 ロール,ピッチ,ヨー
// [15]      ボタンデータ1
// [16]      ボタンアナログ1（Stick Left）
// [17]      ボタンアナログ2（Stick Right）
// [18]      ボタンアナログ3（L2,R2 アナログ）
// [19]      モーション設定（フレーム数）
// [20]      サーボID LO  コマンド
// [21]      サーボID LO  データ値
// [...]     ...
// [48]      サーボID L14 コマンド
// [49]      サーボID L14 データ値
// [50]      サーボID RO  コマンド
// [51]      サーボID RO  データ値
// [...]     ...
// [78]      サーボID R14 コマンド
// [79]      サーボID R14 データ値
// [80]-[MERIDIM90_LEN-3] free (Meridim90では[87]まで)
// [MERIDIM90_LEN-2] ERROR CODE
// [MERIDIM90_LEN-1] チェックサム

#define MERIDIM90_LEN 90                      // Meridim配列の長さ設定（デフォルトは90）
const int MERIDIM90_BYTE = MERIDIM90_LEN * 2; // Meridim配列のバイト型の長さ

enum Meridim90_MasterCommand {
  // マスターコマンド定義
  MCMD_TORQUE_ALL_OFF = 0,             // すべてのサーボトルクをオフにする（脱力）
  MCMD_DUMMY_DATA = -32768,            // SPI送受信用のダミーデータ判定用
  MCMD_TEST_VALUE = -32767,            // テスト用の仮設変数
  MCMD_SENSOR_YAW_CALIB = 10002,       // センサの推定ヨー軸を現在値センターとしてリセット
  MCMD_SENSOR_ALL_CALIB = 10003,       // センサの3軸について現在値を原点としてリセット
  MCMD_ERR_CLEAR_SERVO_ID = 10004,     // 通信エラーのサーボのIDをクリア(MRD_ERR_l)
  MCMD_BOARD_TRANSMIT_ACTIVE = 10005,  // ボードが定刻で送信を行うモード（PC側が受信待ち）
  MCMD_BOARD_TRANSMIT_PASSIVE = 10006, // ボードが受信を待ち返信するモード（PC側が定刻送信）
  MCMD_FRAMETIMER_RESET = 10007,       // フレームタイマーを現在時刻にリセット
  MCMD_BOARD_STOP_DURING = 10008,      // ボードの末端処理を[MRD_STOP_FRAMES]ミリ秒止める
  MCMD_EEPROM_ENTER_WRITE = 10009,     // EEPROM書き込みモードのスタート
  MCMD_EEPROM_EXIT_WRITE = 10010,      // EEPROM書き込みモードの終了
  MCMD_EEPROM_ENTER_READ = 10011,      // EEPROM読み出しモードのスタート
  MCMD_EEPROM_EXIT_READ = 10012,       // EEPROM読み出しモードの終了
  MCMD_SD_CARD_ENTER_WRITE = 10013,    // SD_CARD書き込みモードのスタート
  MCMD_SD_CARD_EXIT_WRITE = 10014,     // SD_CARD書き込みモードの終了
  MCMD_SD_CARD_ENTER_READ = 10015,     // SD_CARD読み出しモードのスタート
  MCMD_SD_CARD_EXIT_READ = 10016,      // SD_CARD読み出しモードの終了
  MCMD_EEPROM_SAVE_TRIM = 10101,       // 現在の姿勢をトリム値としてサーボに書き込む
  MCMD_EEPROM_LOAD_TRIM = 10102,       // EEPROMのトリム値をサーボに反映する
  MCMD_NAK = 32766,                    // コマンド実行の失敗を応答
  MCMD_ACK = 32767,                    // コマンド実行の成功を応答
};
enum Meridim90_ErrorBit {
  // エラービット MRD_ERR_CODEの上位8bit分
  ERRBIT_15_ESP_PC = 15,       // ESP32 → PC のUDP受信エラー (0:エラーなし、1:エラー検出)
  ERRBIT_14_PC_ESP = 14,       // PC → ESP32 のUDP受信エラー
  ERRBIT_13_ESP_TSY = 13,      // ESP32 → TeensyのSPI受信エラー
  ERRBIT_12_TSY_ESP = 12,      // Teensy → ESP32 のSPI受信エラー
  ERRBIT_11_BOARD_DELAY = 11,  // Teensy or ESP32の処理ディレイ (末端で捕捉)
  ERRBIT_10_UDP_ESP_SKIP = 10, // PC → ESP32 のUDPフレームスキップエラー
  ERRBIT_9_BOARD_SKIP = 9,     // PC → ESP32 → Teensy のフレームスキップエラー(末端で捕捉)
  ERRBIT_8_PC_SKIP = 8,        // Teensy → ESP32 → PC のフレームスキップエラー(末端で捕捉)
};

typedef union {
  int16_t mrd_master = Meridim90_MasterCommand::MCMD_TORQUE_ALL_OFF; //	マスターコマンド
  uint16_t mrd_sequential;                                           //	シーケンス番号
  int16_t mrd_acc_x;                                                 //	加速度センサX値
  int16_t mrd_acc_y;                                                 //	加速度センサY値
  int16_t mrd_acc_z;                                                 //	加速度センサZ値
  int16_t mrd_gyro_x;                                                //	ジャイロセンサX値
  int16_t mrd_gyro_y;                                                //	ジャイロセンサY値
  int16_t mrd_gyro_z;                                                //	ジャイロセンサZ値
  int16_t mrd_mag_x;                                                 //	磁気コンパスX値
  int16_t mrd_mag_y;                                                 //	磁気コンパスY値
  int16_t mrd_mag_z;                                                 //	磁気コンパスZ値
  int16_t mrd_temp;                                                  //	温度センサ値
  int16_t mrd_dir_roll;                                              //	DMP推定ロール方向値
  int16_t mrd_dir_pitch;                                             //	DMP推定ピッチ方向値
  int16_t mrd_dir_yaw;                                               //	DMP推定ヨー方向値
  int16_t mrd_control_buttons;                                       //	リモコンの基本ボタン値
  int16_t mrd_control_stick_l;                                       //	リモコンの左スティックアナログ値
  int16_t mrd_control_stick_r;                                       //	リモコンの右スティックアナログ値
  uint16_t mrd_control_l2r2analog;                                   //	リモコンのL2R2ボタンアナログ値
  uint8_t mrd_motion_frames;                                         //	モーション設定のフレーム数
  uint8_t mrd_stop_frames_ms;                                        //	ボード停止時のフレーム数
  int16_t head_y_cmd;                                                //	頭ヨーのコマンド
  int16_t head_y_val;                                                //	頭ヨーの値
  int16_t l_shoulder_p_cmd;                                          //	左肩ピッチのコマンド
  int16_t l_shoulder_p_val;                                          //	左肩ピッチの値
  int16_t l_shoulder_r_cmd;                                          //	左肩ロールのコマンド
  int16_t l_shoulder_r_val;                                          //	左肩ロールの値
  int16_t l_elbow_y_cmd;                                             //	左肘ヨーのコマンド
  int16_t l_elbow_y_val;                                             //	左肘ヨーの値
  int16_t l_elbow_p_cmd;                                             //	左肘ピッチのコマンド
  int16_t l_elbow_p_val;                                             //	左肘ピッチの値
  int16_t l_hipjoint_y_cmd;                                          //	左股ヨーのコマンド
  int16_t l_hipjoint_y_val;                                          //	左股ヨーの値
  int16_t l_hipjoint_r_cmd;                                          //	左股ロールのコマンド
  int16_t l_hipjoint_r_val;                                          //	左股ロールの値
  int16_t l_hipjoint_p_cmd;                                          //	左股ピッチのコマンド
  int16_t l_hipjoint_p_val;                                          //	左股ピッチの値
  int16_t l_knee_p_cmd;                                              //	左膝ピッチのコマンド
  int16_t l_knee_p_val;                                              //	左膝ピッチの値
  int16_t l_ankle_p_cmd;                                             //	左足首ピッチのコマンド
  int16_t l_ankle_p_val;                                             //	左足首ピッチの値
  int16_t l_ankle_r_cmd;                                             //	左足首ロールのコマンド
  int16_t l_ankle_r_val;                                             //	左足首ロールの値
  int16_t l_servo_id11_cmd;                                          //	追加サーボ用のコマンド
  int16_t l_servo_id11_val;                                          //	追加サーボ用の値
  int16_t l_servo_id12_cmd;                                          //	追加サーボ用のコマンド
  int16_t l_servo_id12_val;                                          //	追加サーボ用の値
  int16_t l_servo_id13_cmd;                                          //	追加サーボ用のコマンド
  int16_t l_servo_id13_val;                                          //	追加サーボ用の値
  int16_t l_servo_id14_cmd;                                          //	追加サーボ用のコマンド
  int16_t l_servo_id14_val;                                          //	追加サーボ用の値
  int16_t waist_y_cmd;                                               //	腰ヨーのコマンド
  int16_t waist_y_val;                                               //	腰ヨーの値
  int16_t r_shoulder_p_cmd;                                          //	右肩ピッチのコマンド
  int16_t r_shoulder_p_val;                                          //	右肩ピッチの値
  int16_t r_shoulder_r_cmd;                                          //	右肩ロールのコマンド
  int16_t r_shoulder_r_val;                                          //	右肩ロールの値
  int16_t r_elbow_y_cmd;                                             //	右肘ヨーのコマンド
  int16_t r_elbow_y_val;                                             //	右肘ヨーの値
  int16_t r_elbow_p_cmd;                                             //	右肘ピッチのコマンド
  int16_t r_elbow_p_val;                                             //	右肘ピッチの値
  int16_t r_hipjoint_y_cmd;                                          //	右股ヨーのコマンド
  int16_t r_hipjoint_y_val;                                          //	右股ヨーの値
  int16_t r_hipjoint_r_cmd;                                          //	右股ロールのコマンド
  int16_t r_hipjoint_r_val;                                          //	右股ロールの値
  int16_t r_hipjoint_p_cmd;                                          //	右股ピッチのコマンド
  int16_t r_hipjoint_p_val;                                          //	右股ピッチの値
  int16_t r_knee_p_cmd;                                              //	右膝ピッチのコマンド
  int16_t r_knee_p_val;                                              //	右膝ピッチの値
  int16_t r_ankle_p_cmd;                                             //	右足首ピッチのコマンド
  int16_t r_ankle_p_val;                                             //	右足首ピッチの値
  int16_t r_ankle_r_cmd;                                             //	右足首ロールのコマンド
  int16_t r_ankle_r_val;                                             //	右足首ロールの値
  int16_t r_servo_id11_cmd;                                          //	追加テスト用のコマンド
  int16_t r_servo_id11_val;                                          //	追加テスト用の値
  int16_t r_servo_id12_cmd;                                          //	追加テスト用のコマンド
  int16_t r_servo_id12_val;                                          //	追加テスト用の値
  int16_t r_servo_id13_cmd;                                          //	追加テスト用のコマンド
  int16_t r_servo_id13_val;                                          //	追加テスト用の値
  int16_t r_servo_id14_cmd;                                          //	追加テスト用のコマンド
  int16_t r_servo_id14_val;                                          //	追加テスト用の値
  int16_t mrd_userdata_80;                                           //	ユーザー定義用
  int16_t mrd_userdata_81;                                           //	ユーザー定義用
  int16_t mrd_userdata_82;                                           //	ユーザー定義用
  int16_t mrd_userdata_83;                                           //	ユーザー定義用
  int16_t mrd_userdata_84;                                           //	ユーザー定義用
  int16_t mrd_userdata_85;                                           //	ユーザー定義用
  int16_t mrd_userdata_86;                                           //	ユーザー定義用
  int16_t mrd_userdata_87;                                           //	ユーザー定義用
  uint16_t mrd_err;                                                  //	ERROR CODE
  int16_t mrd_cksm;                                                  //	CHECK SUM
  int16_t cobs1;                                                     //	COBS
  int16_t cobs2;                                                     //	COBS
} Meridim90Union;

#endif // MERIDIM90_HPP
