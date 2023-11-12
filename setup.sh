#!/bin/bash

cd "$(dirname $0)"

wget https://www.a-quest.com/archive/package/aquestalkpi-20220207.tgz
wget -O efficientdet.tflite https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/int8/1/efficientdet_lite0.tflite

tar xvf aquestalkpi-20220207.tgz
cp aquestalkpi/bin64/AquesTalkPi aquestalkpi/AquesTalkPi

source /opt/ros/humble/setup.bash
vcs import < ext.repos
rosdep install -yi --from-paths .
colcon build --symlink-install
