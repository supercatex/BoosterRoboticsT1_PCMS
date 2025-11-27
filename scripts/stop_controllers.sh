#!/bin/bash

PYTHON_SCRIPT_NAME="RobotVoiceController.py"
PID=$(ps aux | grep "$PYTHON_SCRIPT_NAME" | grep -v grep | awk '{print $2}')
if [ -z "$PID" ]; then
    echo "No running process found for $PYTHON_SCRIPT_NAME."
else
    kill -9 $PID
fi 

PYTHON_SCRIPT_NAME="RobotModeController.py"
PID=$(ps aux | grep "$PYTHON_SCRIPT_NAME" | grep -v grep | awk '{print $2}')
if [ -z "$PID" ]; then
    echo "No running process found for $PYTHON_SCRIPT_NAME."
else
    kill -9 $PID
fi 

PYTHON_SCRIPT_NAME="RobotMoveController.py"
PID=$(ps aux | grep "$PYTHON_SCRIPT_NAME" | grep -v grep | awk '{print $2}')
if [ -z "$PID" ]; then
    echo "No running process found for $PYTHON_SCRIPT_NAME."
else
    kill -9 $PID
fi 

PYTHON_SCRIPT_NAME="RobotHeadController.py"
PID=$(ps aux | grep "$PYTHON_SCRIPT_NAME" | grep -v grep | awk '{print $2}')
if [ -z "$PID" ]; then
    echo "No running process found for $PYTHON_SCRIPT_NAME."
else
    kill -9 $PID
fi 

echo "關閉完成"
