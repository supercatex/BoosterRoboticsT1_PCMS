#!/bin/bash

./scripts/stop_controllers.sh

echo "正在啟動語音合成功能..."
python3 ./src/pcms/RobotVoiceController.py &

echo "正在啟動高層服務接口..."
python3 ./src/pcms/HighLevelController.py &

echo "正在啟動頭部控制功能..."
python3 ./src/pcms/RobotHeadController.py &

echo "啟動完成"
