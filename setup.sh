#!/bin/bash

cd "$(dirname $0)"

wget -O aquestalkpi.tgz https://www.a-quest.com/archive/package/aquestalkpi-20220207.tgz
wget -O efficientdet.tflite https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/int8/1/efficientdet_lite0.tflite
wget -O gesture_recognizer.task https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task

tar xvf aquestalkpi.tgz
cp aquestalkpi/bin64/AquesTalkPi aquestalkpi/AquesTalkPi

source /opt/ros/humble/setup.bash
vcs import < ext.repos
rosdep install -yi --from-paths .
colcon build --symlink-install
