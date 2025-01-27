/**
 * @file sample_app.hpp
 * @brief 動作確認用のサンプルアプリケーションです
 * @version 1.2.0
 * @date 2025-01-27
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __SAMPLE_APP_HPP__
#define __SAMPLE_APP_HPP__

#include <Meridim90.hpp>
#include <board/meridian_board_lite.hpp>

bool sample_app_setup(mrd_entity &entity);
bool sample_app_loop(Meridim90 &mrd_meridim, mrd_entity &entity);

#endif // __SAMPLE_APP_HPP__
