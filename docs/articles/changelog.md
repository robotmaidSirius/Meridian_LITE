---
layout: page
title: ChangeLog
permalink: /:basename:output_ext
---

* [Meridian_LITE v1.1.1](https://github.com/Ninagawa123/Meridian_LITE/)

より改良・拡張しやすくするため, 大規模なリファクタリングを行いました.
命名規則はLLVM準拠とし, 内容を "Meridian_LITE_for_ESP32/.clang-format" ファイルにコメントしています.
またコードを構成要素ごとにヘッダーファイルで切り分け, モジュール化することで, 改造や拡張の見通しを立ちやすくしました.
フローチャートもDocsにて公開しています.

ライブラリの関数や変数表など, システムの詳細については以下のサイトがあります. 内容はv1.1.1に合わせました.
[https://ninagawa123.github.io/Meridian_info/](https://ninagawa123.github.io/Meridian_info/)



## バージョン更新履歴

#### 2024.08.18 v1.1.1
コードをモジュールに分割し, Meridian_TWIN v1.1.0 と同等の構成にしました.
命名規則を導入し, 大規模なリファクタリングを行いました.
コードについて, Meridian_TWIN v1.1.1 との共通部分を増やしました.

#### 2024.08.19 v1.0.2
大幅なリファクタリングを施したv1.1.1のリリースにあたり, 旧版の最新版をv1.0.2 としました.

#### 2023.09.15 v1.0.1
\#define ESP32_STDALONE 0 をconfig.hに追加し, 値を1に設定することでESP32単体で通信テストが行えるようにしました.
その際, サーボ値は調べず, 代わりにL0番のサーボ値として+-30度のサインカーブを代入しつづけます.
