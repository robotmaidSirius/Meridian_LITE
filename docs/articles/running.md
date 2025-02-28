---
layout: page
title: ボードとロボットの起動
permalink: /:basename:output_ext
---


これでボード側の準備が整いました.
PCとボードをUSBで接続した状態でボードを起動すると,シリアルモニタに起動時のステータスがメッセージとして表示されます.
ただし,PCとの連携にはPC側でMeridian Consolenを立ち上げておくなどの準備が必要になります.
<br>

## Meridian consoleを実行する
Meridianで受け取るデータを表示できるコンソールを用意しました.python3が使える環境で実行可能です.
下記のリポジトリより, PC側の設定を行い, 実行してください.
https://github.com/Ninagawa123/Meridian_console
<img width="400" alt="Meridian_console_py" src="./images/Meridian_console_py.png">

<br>

## Unity版デモを実行する
Meridian_LITEとUnityを連携させることができます.
下記のリポジトリの内容をお試しください.
[https://github.com/Ninagawa123/Meridian_Unity/tree/main](https://github.com/Ninagawa123/Meridian_Unity/tree/main)

<img width="400" alt="Meridian_Unity" src="./images/Meridian_unity.png">

<br>

## ROS版デモを実行する
Meridian_TWINとUnityを連携させることができます.
下記のリポジトリより「ROS版デモを実行する」をお試しください.
[https://github.com/Ninagawa123/Meridian_TWIN/edit/main/README.md](https://github.com/Ninagawa123/Meridian_TWIN/edit/main/README.md)
<br>

## リモコンの使用方法
**KRR-5FH/KRC5-FH**
config.hの「#define MOUNT_PAD KRR5FH」と設定してボードに書き込みます.
受信機のKRR-5FHはボードの**R系統に接続**します. KRC-5FHのペアリングは製品の説明書の通りです.
受信信号はMeridianに格納されるので, Meridian_console.pyでボタンの受信状況が確認できます.

**WIIリモコン**
v1.1.1 からwiiリモコンをおまけ機能として復活しました. (Meridianの通信速度が若干低下します.)
config.hの「#define MOUNT_PAD WIIMOTE」と設定してボードに書き込み, 起動直後にWiiリモコンの1,2ボタンを両押しするとペアリングが確立します.ヌンチャクのレバーも左側のアナログ十字スティックとして機能します.
また、HOMEボタンがアナログスティックのキャリブレーション（リセット）として機能します.
<br>
