#!/bin/bash

echo "正在啟動語音合成功能..."
python3 ./src/pcms/RobotVoiceController.py &

echo "正在啟動模式控制功能..."
python3 ./src/pcms/RobotModeController.py &

echo "正在啟動移動控制功能..."
python3 ./src/pcms/RobotMoveController.py &

echo "正在啟動頭部控制功能..."
python3 ./src/pcms/RobotHeadController.py &

echo "完成"
