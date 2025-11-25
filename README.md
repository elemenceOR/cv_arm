# CV Arm - Gesture Controlled Robot Arm

A ROS2 package that controls a robot arm in Gazebo using hand gestures detected via OpenCV and MediaPipe, or mouse input for testing.

## Features

- **Gesture Control**: Control robot arm with hand gestures using webcam
- **Mouse Control**: Alternative mouse-based control for testing without camera
- **Real-time Tracking**: MediaPipe hand landmark detection with smoothing
- **ROS2 Integration**: Full ros2_control integration with Gazebo
- **Configurable Camera**: Support for USB cameras and IP camera streams

## System Architecture
<svg viewBox="0 0 800 1000" xmlns="http://www.w3.org/2000/svg">
  <!-- Title -->
  <rect x="50" y="20" width="700" height="60" fill="#2c3e50" rx="5"/>
  <text x="400" y="55" font-family="Arial, sans-serif" font-size="24" font-weight="bold" fill="white" text-anchor="middle">GESTURE CONTROL SYSTEM</text>
  
  <!-- Webcam Camera -->
  <rect x="300" y="110" width="200" height="80" fill="#3498db" stroke="#2c3e50" stroke-width="2" rx="5"/>
  <text x="400" y="140" font-family="Arial, sans-serif" font-size="16" font-weight="bold" fill="white" text-anchor="middle">Webcam</text>
  <text x="400" y="160" font-family="Arial, sans-serif" font-size="16" font-weight="bold" fill="white" text-anchor="middle">Camera</text>
  <text x="400" y="180" font-family="Arial, sans-serif" font-size="12" fill="white" text-anchor="middle">Captures video feed</text>
  
  <!-- Arrow 1 -->
  <line x1="400" y1="190" x2="400" y2="230" stroke="#34495e" stroke-width="3" marker-end="url(#arrowhead)"/>
  <text x="450" y="215" font-family="Arial, sans-serif" font-size="11" fill="#2c3e50" font-style="italic">Video frames</text>
  
  <!-- OpenCV + MediaPipe -->
  <rect x="200" y="230" width="400" height="140" fill="#e74c3c" stroke="#2c3e50" stroke-width="2" rx="5"/>
  <text x="400" y="255" font-family="Arial, sans-serif" font-size="16" font-weight="bold" fill="white" text-anchor="middle">OpenCV + MediaPipe</text>
  <text x="400" y="275" font-family="Arial, sans-serif" font-size="13" fill="white" text-anchor="middle">(gesture_detector.py)</text>
  <text x="220" y="300" font-family="Arial, sans-serif" font-size="12" fill="white">• Detects hand landmarks</text>
  <text x="220" y="320" font-family="Arial, sans-serif" font-size="12" fill="white">• Tracks index finger tip</text>
  <text x="220" y="340" font-family="Arial, sans-serif" font-size="12" fill="white">• Calculates position</text>
  
  <!-- Arrow 2 -->
  <line x1="400" y1="370" x2="400" y2="410" stroke="#34495e" stroke-width="3" marker-end="url(#arrowhead)"/>
  <text x="530" y="395" font-family="Arial, sans-serif" font-size="11" fill="#2c3e50" font-style="italic">Index finger Y position (0.0 - 1.0)</text>
  
  <!-- Position Mapping -->
  <rect x="200" y="410" width="400" height="120" fill="#9b59b6" stroke="#2c3e50" stroke-width="2" rx="5"/>
  <text x="400" y="435" font-family="Arial, sans-serif" font-size="16" font-weight="bold" fill="white" text-anchor="middle">Position Mapping</text>
  <text x="220" y="460" font-family="Arial, sans-serif" font-size="12" fill="white">top    (y=0.0)  → 0.1 rad    Elbow bent</text>
  <text x="220" y="480" font-family="Arial, sans-serif" font-size="12" fill="white">middle (y=0.5)  → 1.4 rad    Elbow halfway</text>
  <text x="220" y="500" font-family="Arial, sans-serif" font-size="12" fill="white">bottom (y=1.0)  → 2.7 rad    Elbow extended</text>
  
  <!-- Arrow 3 -->
  <line x1="400" y1="530" x2="400" y2="570" stroke="#34495e" stroke-width="3" marker-end="url(#arrowhead)"/>
  <text x="510" y="555" font-family="Arial, sans-serif" font-size="11" fill="#2c3e50" font-style="italic">Float64MultiArray message</text>
  
  <!-- ROS2 Topic -->
  <rect x="200" y="570" width="400" height="80" fill="#16a085" stroke="#2c3e50" stroke-width="2" rx="5"/>
  <text x="400" y="595" font-family="Arial, sans-serif" font-size="16" font-weight="bold" fill="white" text-anchor="middle">ROS2 Topic</text>
  <text x="400" y="615" font-family="Arial, sans-serif" font-size="12" fill="white" text-anchor="middle">/elbow_position_controller/</text>
  <text x="400" y="635" font-family="Arial, sans-serif" font-size="12" fill="white" text-anchor="middle">commands</text>
  
  <!-- Arrow 4 -->
  <line x1="400" y1="650" x2="400" y2="690" stroke="#34495e" stroke-width="3" marker-end="url(#arrowhead)"/>
  <text x="480" y="675" font-family="Arial, sans-serif" font-size="11" fill="#2c3e50" font-style="italic">Position command</text>
  
  <!-- ROS2 Control -->
  <rect x="200" y="690" width="400" height="120" fill="#f39c12" stroke="#2c3e50" stroke-width="2" rx="5"/>
  <text x="400" y="715" font-family="Arial, sans-serif" font-size="16" font-weight="bold" fill="white" text-anchor="middle">ROS2 Control</text>
  <text x="400" y="735" font-family="Arial, sans-serif" font-size="13" fill="white" text-anchor="middle">(JointGroupPositionController)</text>
  <text x="220" y="760" font-family="Arial, sans-serif" font-size="12" fill="white">• Receives target position</text>
  <text x="220" y="780" font-family="Arial, sans-serif" font-size="12" fill="white">• Plans trajectory</text>
  <text x="220" y="800" font-family="Arial, sans-serif" font-size="12" fill="white">• Sends to hardware interface</text>
  
  <!-- Arrow 5 -->
  <line x1="400" y1="810" x2="400" y2="850" stroke="#34495e" stroke-width="3" marker-end="url(#arrowhead)"/>
  <text x="470" y="835" font-family="Arial, sans-serif" font-size="11" fill="#2c3e50" font-style="italic">Joint commands</text>
  
  <!-- Gazebo Simulation -->
  <rect x="200" y="850" width="400" height="120" fill="#27ae60" stroke="#2c3e50" stroke-width="2" rx="5"/>
  <text x="400" y="875" font-family="Arial, sans-serif" font-size="16" font-weight="bold" fill="white" text-anchor="middle">Gazebo Simulation</text>
  <text x="400" y="895" font-family="Arial, sans-serif" font-size="13" fill="white" text-anchor="middle">(Physics Engine)</text>
  <text x="220" y="920" font-family="Arial, sans-serif" font-size="12" fill="white">• Updates joint position</text>
  <text x="220" y="940" font-family="Arial, sans-serif" font-size="12" fill="white">• Simulates dynamics</text>
  <text x="220" y="960" font-family="Arial, sans-serif" font-size="12" fill="white">• Renders visualization</text>
  
  <!-- Arrow marker definition -->
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="10" refX="9" refY="3" orient="auto">
      <polygon points="0 0, 10 3, 0 6" fill="#34495e"/>
    </marker>
  </defs>
</svg>

## Data Flow Details

1. CAPTURE PHASE
   Camera → 640x480 RGB frame @ 30 FPS

2. DETECTION PHASE  
   MediaPipe Hand Detector →
   - 21 hand landmarks in 3D
   - Confidence scores
   - Handedness (left/right)

3. PROCESSING PHASE
   landmark[8] (index finger tip) →
   normalized_y = 1.0 - tip.y  (invert, because y increases downward)
   
4. MAPPING PHASE
   target_angle = joint_min + normalized_y * (joint_max - joint_min)
   0.1 rad ≤ angle ≤ 2.721 rad

5. SMOOTHING PHASE
   current_angle = α * target_angle + (1-α) * current_angle
   (α = 0.3 for smooth motion)

6. PUBLISHING PHASE
   msg.data = [current_angle]
   publisher.publish(msg)

7. CONTROL PHASE
   Controller receives command →
   Plans trajectory →
   Sends to Gazebo

8. SIMULATION PHASE
   Gazebo updates physics →
   Robot arm moves →
   Visual feedback in simulation

## ROS2 Node Graph


## Message Types

### Float64MultiArray
```
std_msgs/Float64MultiArray
├── layout (MultiArrayLayout)
│   ├── dim[] (MultiArrayDimension)
│   └── data_offset (uint32)
└── data[] (float64)
    └── [0]: joint_angle (radians)
```

### JointState
```
sensor_msgs/JointState
├── header (Header)
├── name[] (string)
│   └── ["elbow_joint"]
├── position[] (float64)
│   └── [current_angle]
├── velocity[] (float64)
└── effort[] (float64)
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

### Custom Camera Source

If ROS is used in a virtual machine or Ubuntu WSL, accesing the inbuild webcamera or usb camers is difficult. The solution is to use a IP camera and connect to it for the recognition part. 

## Nodes

### `gesture_detector`
Detects hand gestures and publishes joint commands.

**Parameters:**
- `camera_source`
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

