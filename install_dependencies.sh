#!/bin/bash

echo "================================================"
echo "CV Arm - Gesture Control Setup"
echo "================================================"
echo ""

echo "Step 1: Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y python3-pip python3-opencv

echo ""
echo "Step 2: Installing MediaPipe..."
pip3 install mediapipe

echo ""
echo "Step 3: Building ROS2 package..."
cd /home/sadeep/ros2_ws
colcon build --packages-select cv_arm
source install/setup.bash

echo ""
echo "================================================"
echo "Setup Complete!"
echo "================================================"
echo ""
echo "To run the gesture control system:"
echo "  ros2 launch cv_arm gesture_control.launch.py"
echo ""
echo "Or run components separately:"
echo "  Terminal 1: ros2 launch cv_arm spawn_elbow.launch.py"
echo "  Terminal 2: ros2 run cv_arm gesture_detector"
echo ""
