/**
 * @file application/mrd_app.hpp
 * @brief アプリケーション用のヘッダファイル
 * @version 1.2.0
 * @date 2025-03-03
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef MRD_APP_HPP
#define MRD_APP_HPP

#include "config.h"
#include <Meridian.h>               // Meridianのライブラリ導入
#include <mrd_module/sv_common.hpp> // サーボ用定義

extern MERIDIANFLOW::Meridian mrd;
extern ServoParam sv;

////////////////////////////////////////////////////////////////////////////////////////////////////
// 関数: EEPROM
////////////////////////////////////////////////////////////////////////////////////////////////////
void app_eeprom_setup();
bool mrd_set_eeprom();
void mrd_get_eeprom();

#endif // MRD_APP_HPP
