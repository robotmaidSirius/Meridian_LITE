#ifndef __MERIDIAN_MESSAGE_H__
#define __MERIDIAN_MESSAGE_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"

// ライブラリ導入
#include <WiFi.h>

//================================================================================================================
//  シリアルモニタ表示用の関数
//================================================================================================================

class MrdMsgHandler {
private:
  Stream &m_serial; // シリアルオブジェクトの参照を保持

public:
  // コンストラクタでStreamオブジェクトを受け取り, メンバーに保存
  MrdMsgHandler(Stream &a_serial) : m_serial(a_serial) {}

  //------------------------------------------------------------------------------------
  //  起動時メッセージ
  //------------------------------------------------------------------------------------

  /// @brief 指定されたUARTラインのサーボIDを表示する.
  /// @param a_label UARTラインのラベル.
  /// @param a_max サーボの最大数.
  /// @param a_mount サーボのマウント状態を示す配列.
  /// @param a_id サーボIDの配列.
  void print_servo_ids(const char *a_label, int a_max, int *a_mount, const int *a_id) {
    m_serial.print(a_label);
    for (int i = 0; i <= a_max; i++) {
      if (a_mount[i] != 0) {
        if (a_id[i] < 10) {
          m_serial.print(" ");
        }
        m_serial.print(a_id[i]);
      } else {
        m_serial.print("__");
      }
      m_serial.print(" ");
    }
    m_serial.println();
  }

  /// @brief マウントされているサーボのIDを表示する.
  /// @param a_sv サーボパラメータの構造体.
  void servo_mounts_2lines(ServoParam a_sv) {
    print_servo_ids("UART_L Servos mounted: ", a_sv.num_max, a_sv.ixl_mount, a_sv.ixl_id);
    print_servo_ids("UART_R Servos mounted: ", a_sv.num_max, a_sv.ixr_mount, a_sv.ixr_id);
  }

  /// @brief wifiの接続開始メッセージを出力する.
  /// @param a_ssid 接続先のSSID.
  void esp_wifi(const char *a_ssid) {
    m_serial.println("WiFi connecting to => " + String(a_ssid)); // WiFi接続完了通知
  }

  /// @brief wifiの接続完了メッセージと各IPアドレスを出力する.
  /// @param a_flg_fixed_ip 固定IPかどうか. true:固定IP, false:動的IP.
  /// @param a_ssid 接続先のSSID.
  /// @param a_fixedip 固定IPの場合の値.
  void esp_ip(bool a_flg_fixed_ip, const char *a_ssid, const char *a_fixedip) {
    m_serial.println("WiFi successfully connected.");                      // WiFi接続完了通知
    m_serial.println("PC's IP address target => " + String(WIFI_SEND_IP)); // 送信先PCのIPアドレスの表示

    if (a_flg_fixed_ip) {
      m_serial.println("ESP32's IP address => " + String(FIXED_IP_ADDR) + " (*Fixed)"); // ESP32自身のIPアドレスの表示
    } else {
      m_serial.print("ESP32's IP address => "); // ESP32自身のIPアドレスの表示
      m_serial.println(WiFi.localIP().toString());
    }
  }

  /// @brief システムの動作開始を示すメッセージを出力する.
  void flow_start_lite_esp() {
    m_serial.println();
    m_serial.println("-) Meridian -LITE- system on ESP32 now flows. (-");
  }

  //------------------------------------------------------------------------------------
  //  イベントメッセージ
  //------------------------------------------------------------------------------------

  /// @brief システム内の様々な通信エラーとスキップ数をモニタリングし, シリアルポートに出力する.
  /// @param mrd_disp_all_err モニタリング表示のオンオフ.
  /// @param a_err エラーデータの入った構造体.
  /// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
  bool all_err(bool mrd_disp_all_err, MrdErr a_err) {
    if (mrd_disp_all_err) {
      m_serial.print("[ERR] es>pc:");
      m_serial.print(a_err.esp_pc);
      m_serial.print(" pc>es:");
      m_serial.print(a_err.pc_esp);
      m_serial.print(" es>ts:");
      m_serial.print(a_err.esp_tsy);
      m_serial.print(" ts>es:");
      m_serial.print(a_err.esp_tsy);
      m_serial.print(" tsSkp:");
      m_serial.print(a_err.tsy_skip);
      m_serial.print(" esSkp:");
      m_serial.print(a_err.esp_skip);
      m_serial.print(" pcSkp:");
      m_serial.print(a_err.pc_skip);
      m_serial.println();
      return true;
    }
    return false;
  }

  /// @brief サーボモーターのエラーを検出した場合にエラーメッセージを表示する.
  /// @param a_line サーボモーターが接続されているUARTライン(L, R, C).
  /// @param a_num エラーが発生しているサーボの番号.
  /// @param a_flg_disp エラーメッセージを表示するかどうかのブール値.
  /// @return エラーメッセージが表示された場合はtrueを, 表示されなかった場合はfalseを返す.
  bool servo_err(UartLine a_line, int a_num, bool a_flg_disp) {
    if (a_flg_disp) {
      m_serial.print("Found servo err ");
      if (a_line == L) {
        m_serial.print("L_");
        m_serial.println(a_num);
        return true;
      } else if (a_line == R) {
        m_serial.print("R_");
        m_serial.println(a_num);
        return true;
      } else if (a_line == C) {
        m_serial.print("C_");
        m_serial.println(a_num);
        return true;
      }
    }
    return false;
  }

  /// @brief 期待するシーケンス番号と実施に受信したシーケンス番号を表示する.
  /// @param a_seq_expect 期待するシーケンス番号.
  /// @param a_seq_rcvd 実際に受信したシーケンス番号.
  /// @param a_disp_seq_num 表示するかどうかのブール値.
  /// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
  bool seq_number(uint16_t a_seq_expect, uint16_t a_seq_rcvd, bool a_disp) {
    if (a_disp) {
      m_serial.print("Seq ep/rv ");
      m_serial.print(a_seq_expect);
      m_serial.print("/");
      m_serial.println(a_seq_rcvd);
      return true;
    }
    return false;
  }
};

#endif // __MERIDIAN_MESSAGE_H__
