## ROS package for Common Control Message for Contruction Machinery(COM3)
com3_rosはCOM3をROSで利用するために必要なパッケージをまとめたものです。
COM3用に定義したメッセージ型や建設機械への入出力用ドライバなどを格納しています。

### インストール
- 事前に[COM3](https://github.com/pwri-opera/com3)をインストールしてください
- catkin_ws/src以下でこのリポジトリをクローンして、ビルドしてください
  - cd ~/catkin_ws/src
  - git clone https://github.com/pwri-opera/com3_ros
  - catkin build
  - souce ~/catkin_ws/devel/setup.bash

