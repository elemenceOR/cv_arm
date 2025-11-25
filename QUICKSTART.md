# Quick Start Guide - Gesture Controlled Robot Arm

## 📋 Prerequisites Checklist

- [ ] ROS2 installed (Jazzy or Humble)
- [ ] Gazebo installed
- [ ] Webcam connected
- [ ] Python 3.8+

## 🚀 Installation (One-Time Setup)

### Step 1: Install System Dependencies
```bash
sudo apt-get update
sudo apt-get install python3-pip python3-opencv
```

### Step 2: Install MediaPipe
```bash
pip3 install mediapipe
```

### Step 3: Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select cv_arm
source install/setup.bash
```

### Step 4: Verify Installation
```bash
python3 ~/ros2_ws/src/cv_arm/test_setup.py
```

## ▶️ Running the System

### Option A: Launch Everything at Once (Recommended)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cv_arm gesture_control.launch.py
```

**Wait for:**
1. Gazebo window to open
2. Robot to spawn (blue and red arm)
3. OpenCV window showing camera feed

### Option B: Launch Components Separately

**Terminal 1 - Gazebo & Robot:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cv_arm spawn_elbow.launch.py
```

**Terminal 2 - Gesture Control:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run cv_arm gesture_detector
```

## 🎮 How to Control

1. **Position yourself** in front of the camera
2. **Show your hand** with palm facing the camera
3. **Raise index finger up** → Robot elbow bends
4. **Lower index finger down** → Robot elbow extends
5. **Press 'q'** in OpenCV window to quit

### Tips for Best Results:
- ✓ Good lighting
- ✓ Plain background
- ✓ Keep hand in frame
- ✓ Smooth movements (system has smoothing built-in)

## 🔍 Debugging

### Check if Controllers are Running
```bash
ros2 control list_controllers
```
Expected output:
```
elbow_position_controller[position_controllers/JointGroupPositionController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### Check Topics
```bash
ros2 topic list
```
Should see:
- `/elbow_position_controller/commands`
- `/joint_states`

### Monitor Joint Commands
```bash
ros2 topic echo /elbow_position_controller/commands
```

### Monitor Joint States
```bash
ros2 topic echo /joint_states
```

### Test Camera
```bash
ls /dev/video*
python3 -c "import cv2; cap=cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera FAIL')"
```

## 🛠️ Common Issues

### Issue: "Cannot open camera"
**Solution:**
```bash
# Check camera devices
ls /dev/video*

# Give permissions
sudo chmod 666 /dev/video0

# Try different camera index in gesture_detector.py (line 40):
self.cap = cv2.VideoCapture(1)  # Try 1, 2, 3...
```

### Issue: "Import mediapipe could not be resolved"
**Solution:**
```bash
pip3 install --user mediapipe
# or
python3 -m pip install mediapipe
```

### Issue: Controller not responding
**Solution:**
```bash
# Restart controllers
ros2 control set_controller_state elbow_position_controller inactive
ros2 control set_controller_state elbow_position_controller active
```

### Issue: Robot moves too fast/jerky
**Solution:** Edit `cv_arm/gesture_detector.py` line ~50:
```python
self.smoothing = 0.1  # Lower value = smoother (0.1-0.5)
```

### Issue: Gesture not detected
**Solutions:**
- Ensure palm is facing camera
- Move hand closer to camera
- Improve lighting
- Lower confidence threshold in code (line ~38):
```python
min_detection_confidence=0.3,  # Lower from 0.5
```

## 📊 System Monitor

Open multiple terminals to monitor:

**Terminal 1 - Joint States:**
```bash
ros2 topic echo /joint_states
```

**Terminal 2 - Commands:**
```bash
ros2 topic echo /elbow_position_controller/commands
```

**Terminal 3 - Node Graph:**
```bash
rqt_graph
```

**Terminal 4 - TF Tree:**
```bash
ros2 run tf2_tools view_frames
```

## 🎯 Testing Individual Components

### Test 1: Manual Joint Control
```bash
# Send direct command
ros2 topic pub --once /elbow_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.5]}"
```

### Test 2: Gesture Detection Only
```bash
# Run gesture detector and watch output
ros2 run cv_arm gesture_detector
# Watch the angle values in terminal
```

### Test 3: Controller Response
```bash
# Terminal 1: Monitor states
ros2 topic echo /joint_states

# Terminal 2: Send commands
ros2 topic pub /elbow_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}"
```

## 🔧 Customization Quick Edits

### Change Joint Limits
File: `model/arm.urdf.xacro` (line ~103)
```xml
<limit lower="0.1" upper="2.721" effort="100.0" velocity="1.0"/>
```

### Change Control Rate
File: `cv_arm/gesture_detector.py` (line ~53)
```python
self.timer = self.create_timer(0.033, self.process_frame)  # 0.033 = 30Hz
```

### Change Smoothing
File: `cv_arm/gesture_detector.py` (line ~50)
```python
self.smoothing = 0.3  # 0.1 = very smooth, 0.9 = very responsive
```

### Use Different Finger
File: `cv_arm/gesture_detector.py` (line ~90)
```python
index_tip = hand_landmarks.landmark[12]  # Use middle finger instead
```

## 📚 Learning Resources

### ROS2 Control
- https://control.ros.org/

### MediaPipe Hands
- https://google.github.io/mediapipe/solutions/hands.html

### Gazebo
- https://gazebosim.org/

## 🎓 Next Steps

1. **Add More Joints** - Extend to full 6-DOF arm
2. **Add Gripper** - Control gripper with pinch gesture
3. **Add Obstacles** - Spawn objects in Gazebo
4. **Record Trajectories** - Save and replay motions
5. **Multi-Hand Control** - Control multiple joints simultaneously
6. **Add Vision** - Add camera to robot for visual servoing

## 📝 Quick Command Reference

```bash
# Build
colcon build --packages-select cv_arm

# Source
source install/setup.bash

# Launch
ros2 launch cv_arm gesture_control.launch.py

# Run standalone
ros2 run cv_arm gesture_detector

# List nodes
ros2 node list

# List topics
ros2 topic list

# Echo topic
ros2 topic echo /joint_states

# Info about topic
ros2 topic info /elbow_position_controller/commands

# Manual control
ros2 topic pub /elbow_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.5]}"

# Kill all nodes
pkill -9 -f ros2
```

## 🎬 Demo Video Script

1. Launch system: `ros2 launch cv_arm gesture_control.launch.py`
2. Wait for Gazebo and OpenCV windows
3. Show hand to camera
4. Move index finger up slowly → Elbow bends
5. Move index finger down slowly → Elbow extends
6. Do quick up/down movements → Show smoothing
7. Press 'q' to exit

---

**Happy Controlling! 🤖✋**
