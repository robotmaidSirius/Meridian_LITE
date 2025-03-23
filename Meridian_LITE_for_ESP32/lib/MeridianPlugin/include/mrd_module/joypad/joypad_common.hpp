/**
 * @file joypad_common.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-03-23
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef JOYPAD_COMMON_HPP
#define JOYPAD_COMMON_HPP

enum PadType {   // リモコン種の列挙型(NONE, PC, MERIMOTE, BLUERETRO, SBDBT, KRR5FH)
  NONE = 0,      // リモコンなし
  PC = 0,        // PCからのPD入力情報を使用
  MERIMOTE = 1,  // MERIMOTE(未導入)
  BLUERETRO = 2, // BLUERETRO(未導入)
  SBDBT = 3,     // SBDBT(未導入)
  KRR5FH = 4,    // KRR5FH
  WIIMOTE = 5,   // WIIMOTE / WIIMOTE + Nunchuk
  WIIMOTE_C = 6, // WIIMOTE+Classic
};

#endif
