# CV Arm - Gesture Controlled Robot Arm

A ROS2 package that controls a robot arm in Gazebo using hand gestures detected via OpenCV and MediaPipe, or mouse input for testing.

## Features

- **Gesture Control**: Control robot arm with hand gestures using webcam
- **Mouse Control**: Alternative mouse-based control for testing without camera
- **Real-time Tracking**: MediaPipe hand landmark detection with smoothing
- **ROS2 Integration**: Full ros2_control integration with Gazebo
- **Configurable Camera**: Support for USB cameras and IP camera streams

## System Architecture

```
Camera/Mouse → OpenCV/MediaPipe → Detector Node → ROS2 Control → Gazebo
```

For detailed system flow, see [ARCHITECTURE.md](ARCHITECTURE.md).

## Quick Start

See [QUICKSTART.md](QUICKSTART.md) for step-by-step instructions.

### Automated Installation

```bash
cd ~/ros2_ws/src/cv_arm
chmod +x install_dependencies.sh
./install_dependencies.sh
```

## Dependencies

### System Dependencies
```bash
sudo apt-get install python3-pip python3-opencv
pip3 install mediapipe
```

### ROS2 Dependencies
- rclpy
- std_msgs
- sensor_msgs
- cv_bridge
- ros_gz_sim
- ros_gz_bridge
- ros_gz_image
- ros2_control

## Manual Installation

1. Install dependencies:
```bash
sudo apt-get update
sudo apt-get install python3-pip python3-opencv
pip3 install mediapipe
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select cv_arm
source install/setup.bash
```

3. Verify installation:
```bash
python3 ~/ros2_ws/src/cv_arm/test_setup.py
```

## Usage

### Gesture Control (Recommended)

Launch the full system with gesture control:
```bash
ros2 launch cv_arm gesture_control.launch.py
```

This starts:
- Gazebo simulator
- Elbow robot model
- Controllers (joint_state_broadcaster, elbow_position_controller)
- Gesture detection node

### Mouse Control (Testing/Debugging)

Use mouse for control without camera:
```bash
# Terminal 1: Launch robot
ros2 launch cv_arm spawn_elbow.launch.py

# Terminal 2: Start mouse controller
ros2 run cv_arm mouse_controller
```

Move your mouse up/down in the OpenCV window to control the arm.

### Manual Component Launch

Start components separately for debugging:

1. Launch robot in Gazebo:
```bash
ros2 launch cv_arm spawn_elbow.launch.py
```

2. Start gesture detector:
```bash
ros2 run cv_arm gesture_detector
```

### Custom Camera Source

Use IP camera or different webcam:
```bash
# IP camera (e.g., DroidCam)
ros2 run cv_arm gesture_detector --ros-args -p camera_source:="http://192.168.0.249:8080/video"

# Different USB camera
ros2 run cv_arm gesture_detector --ros-args -p camera_source:=2
```

## How to Control

### Gesture Control
1. **Show your hand** to the camera (palm facing camera)
2. **Move your index finger vertically**:
   - Hand at **top** → Elbow **bent** (~0.1 rad)
   - Hand at **middle** → Elbow **halfway** (~1.4 rad)
   - Hand at **bottom** → Elbow **extended** (~2.7 rad)
3. Press **'q'** to quit

**Tips for best results:**
- Use good lighting
- Plain background
- Keep hand fully visible in frame
- Smooth movements (built-in smoothing with α=0.3)

### Mouse Control
- Move mouse **up** → Elbow bends
- Move mouse **down** → Elbow extends
- Press **'q'** to quit

## Nodes

### `gesture_detector`
Detects hand gestures and publishes joint commands.

**Parameters:**
- `camera_source` (default: `http://192.168.0.249:8080/video`)
  - Integer for USB camera (e.g., 0, 1, 2)
  - String URL for IP camera

**Published Topics:**
- `/elbow_position_controller/commands` (Float64MultiArray)

### `mouse_controller`
Alternative controller using mouse input.

**Published Topics:**
- `/elbow_position_controller/commands` (Float64MultiArray)

## Topics and Services

### Published Topics
- `/elbow_position_controller/commands` - Float64MultiArray
  - Published by gesture_detector or mouse_controller
  - Commands the elbow joint position (in radians)

### Subscribed Topics
- `/joint_states` - sensor_msgs/JointState
  - Current state of all robot joints
  - Published by joint_state_broadcaster

### Controllers
- `elbow_position_controller` - JointGroupPositionController
- `joint_state_broadcaster` - JointStateBroadcaster

## File Structure

```
cv_arm/
├── cv_arm/
│   ├── __init__.py
│   ├── gesture_detector.py      # Hand gesture detection node
│   └── mouse_controller.py      # Mouse control node
├── launch/
│   ├── gesture_control.launch.py  # Complete system launcher
│   └── spawn_elbow.launch.py      # Robot spawn launcher
├── model/
│   ├── arm.urdf                   # Generated URDF
│   └── arm.urdf.xacro             # Robot model definition
├── config/
│   └── elbow_controllers.yaml     # Controller configuration
├── ARCHITECTURE.md                # Detailed system architecture
├── QUICKSTART.md                  # Quick start guide
├── install_dependencies.sh        # Automated setup script
├── test_setup.py                  # Installation verification
└── README.md                      # This file
```

## Customization

### Adjust Gesture Sensitivity

Edit `cv_arm/gesture_detector.py`:
```python
# Line ~50: Change smoothing factor
self.smoothing = 0.3  # Lower = smoother, Higher = more responsive
```

### Adjust Mouse Sensitivity

Edit `cv_arm/mouse_controller.py`:
```python
# Line ~18: Change smoothing factor
self.smoothing = 0.3  # Lower = smoother, Higher = more responsive
```

### Change Joint Limits

Edit `model/arm.urdf.xacro`:
```xml
<limit lower="0.1" upper="2.721" effort="100.0" velocity="1.0"/>
```

### MediaPipe Detection Parameters

Edit `cv_arm/gesture_detector.py`:
```python
# Lines ~36-39
self.hands = self.mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,  # Lower = more detections, more false positives
    min_tracking_confidence=0.5    # Lower = more responsive, more jitter
)
```

### Change Default Camera

Edit `cv_arm/gesture_detector.py`:
```python
# Line ~18: Change default camera source
self.declare_parameter('camera_source', 0)  # Use USB camera 0
```

Or use launch parameter:
```bash
ros2 run cv_arm gesture_detector --ros-args -p camera_source:=0
```

### Use Different Gesture

Modify `process_frame()` in `gesture_detector.py`:
- Use different landmarks (see MediaPipe Hand Landmarks below)
- Track hand orientation instead of position
- Detect specific hand poses (fist, peace sign, etc.)

## MediaPipe Hand Landmarks Reference

```
Landmarks (21 total):
    0: Wrist
    1-4: Thumb (CMC, MCP, IP, TIP)
    5-8: Index finger (MCP, PIP, DIP, TIP)
    9-12: Middle finger (MCP, PIP, DIP, TIP)
    13-16: Ring finger (MCP, PIP, DIP, TIP)
    17-20: Pinky (MCP, PIP, DIP, TIP)

Common landmarks:
    8: Index finger tip (used in current implementation)
    4: Thumb tip
    12: Middle finger tip
    16: Ring finger tip
    20: Pinky tip
```

For visual reference: [MediaPipe Hands Documentation](https://google.github.io/mediapipe/solutions/hands.html)

## Troubleshooting

### Camera Issues

**Camera not detected:**
```bash
# List available cameras
ls /dev/video*

# Test camera
python3 -c "import cv2; print(cv2.VideoCapture(0).isOpened())"

# Try different camera indices
ros2 run cv_arm gesture_detector --ros-args -p camera_source:=0  # or 1, 2, etc.
```

**Permission denied:**
```bash
sudo usermod -a -G video $USER
# Log out and back in
```

### Controller Issues

**Controllers not loading:**
```bash
# Check controller status
ros2 control list_controllers

# Expected output:
# elbow_position_controller[position_controllers/JointGroupPositionController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

**Manually load controllers:**
```bash
ros2 control load_controller elbow_position_controller
ros2 control set_controller_state elbow_position_controller active
```

### Gesture Detection Issues

**Hand not detected:**
- Ensure good lighting (avoid backlighting)
- Keep hand within camera frame
- Show palm clearly to camera
- Check MediaPipe confidence thresholds in code
- Try adjusting `min_detection_confidence` (lower = easier detection)

**Jittery movement:**
- Increase smoothing factor: `self.smoothing = 0.1` (smoother but slower)
- Increase `min_tracking_confidence` in MediaPipe settings

**Laggy response:**
- Decrease smoothing factor: `self.smoothing = 0.5` (faster but more jittery)
- Reduce camera resolution
- Check CPU usage

### Gazebo Issues

**Gazebo crashes:**
```bash
# Reset Gazebo
killall gzserver gzclient
```

**Robot not visible:**
- Wait 5-10 seconds for Gazebo to fully load
- Check terminal for error messages
- Verify URDF file exists: `ls ~/ros2_ws/src/cv_arm/model/`

### Build Issues

**Package not found:**
```bash
cd ~/ros2_ws
source install/setup.bash
colcon build --packages-select cv_arm
```

**Missing dependencies:**
```bash
# Re-run dependency installation
cd ~/ros2_ws/src/cv_arm
./install_dependencies.sh
```

## Advanced Features

### Add More Joints

1. Edit `model/arm.urdf.xacro`:
   - Add additional `<joint>` and `<link>` elements
   - Define new revolute or prismatic joints

2. Update `config/elbow_controllers.yaml`:
   - Add new joints to controller configuration
   - Adjust controller parameters

3. Modify gesture detector:
   - Use multiple hand landmarks for different joints
   - Map different gestures to different joints
   - Example: index finger → elbow, middle finger → wrist

### Alternative Input Methods

Replace camera/mouse with:

**Keyboard Input:**
```python
# Add to gesture_detector.py
key = cv2.waitKey(1) & 0xFF
if key == ord('w'):
    target_angle += 0.1
elif key == ord('s'):
    target_angle -= 0.1
```

**Leap Motion Controller:**
- Use Leap Motion SDK
- Track hand position in 3D
- Map to joint angles

**Network Commands:**
- Create ROS2 service for position commands
- Control via web interface
- Remote control over network

**Joystick/Gamepad:**
- Use `sensor_msgs/Joy`
- Map joystick axes to joints
- Button controls for presets

### Multi-Joint Control Example

```python
# Control 3 joints with different fingers
landmarks = results.multi_hand_landmarks[0].landmark

# Index finger → Joint 1 (elbow)
index_y = 1.0 - landmarks[8].y
angle1 = self.joint_min + index_y * (self.joint_max - self.joint_min)

# Middle finger → Joint 2
middle_y = 1.0 - landmarks[12].y
angle2 = self.joint_min + middle_y * (self.joint_max - self.joint_min)

# Ring finger → Joint 3
ring_y = 1.0 - landmarks[16].y
angle3 = self.joint_min + ring_y * (self.joint_max - self.joint_min)

msg.data = [angle1, angle2, angle3]
```

### Add Gripper Control

Use hand open/close gesture to control gripper:
```python
# Calculate distance between thumb tip and index tip
thumb_tip = landmarks[4]
index_tip = landmarks[8]
distance = ((thumb_tip.x - index_tip.x)**2 + 
            (thumb_tip.y - index_tip.y)**2)**0.5

# Map to gripper
gripper_position = distance * gripper_range
```

## Testing

Run the included test to verify setup:
```bash
python3 ~/ros2_ws/src/cv_arm/test_setup.py
```

This checks:
- Python version
- OpenCV installation
- MediaPipe installation
- Camera availability
- ROS2 package build status

## Performance Tips

- **Reduce camera resolution** for better performance:
  ```python
  self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
  self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
  ```

- **Adjust MediaPipe model complexity** (0=lite, 1=full):
  ```python
  self.hands = self.mp_hands.Hands(model_complexity=0)
  ```

- **Skip frames** if processing is slow:
  ```python
  if frame_count % 2 == 0:  # Process every other frame
      results = self.hands.process(rgb_image)
  ```

## Documentation

- [QUICKSTART.md](QUICKSTART.md) - Quick start guide
- [ARCHITECTURE.md](ARCHITECTURE.md) - Detailed system architecture
- [MediaPipe Hands](https://google.github.io/mediapipe/solutions/hands.html) - Hand tracking documentation
- [ROS2 Control](https://control.ros.org/) - ros2_control documentation

## Contributing

Contributions are welcome! Areas for improvement:
- Additional gesture recognition patterns
- Multi-joint coordinated control
- Trajectory recording and playback
- Integration with real hardware
- Machine learning for custom gestures

## License

Apache-2.0

## Author

Created by sadeep

## Acknowledgments

- MediaPipe for hand tracking
- ROS2 community
- Gazebo simulation environment
