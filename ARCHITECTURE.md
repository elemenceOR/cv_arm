# CV Arm System Flow Diagram

## Complete System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         GESTURE CONTROL SYSTEM                           │
└─────────────────────────────────────────────────────────────────────────┘

┌──────────────┐
│   Webcam     │  
│   Camera     │  Captures video feed
└──────┬───────┘
       │
       │ Video frames
       ↓
┌─────────────────────────────┐
│   OpenCV + MediaPipe        │
│   (gesture_detector.py)     │  
│                             │
│  • Detects hand landmarks   │
│  • Tracks index finger tip  │
│  • Calculates position      │
└──────────┬──────────────────┘
           │
           │ Index finger Y position (0.0 - 1.0)
           ↓
┌──────────────────────────────┐
│   Position Mapping           │
│                              │
│  top    (y=0.0)  → 0.1 rad   │  Elbow bent
│  middle (y=0.5)  → 1.4 rad   │  Elbow halfway
│  bottom (y=1.0)  → 2.7 rad   │  Elbow extended
└──────────┬───────────────────┘
           │
           │ Float64MultiArray message
           ↓
┌────────────────────────────────────┐
│   ROS2 Topic                       │
│   /elbow_position_controller/      │
│   commands                         │
└──────────┬─────────────────────────┘
           │
           │ Position command
           ↓
┌────────────────────────────────────┐
│   ROS2 Control                     │
│   (JointGroupPositionController)   │
│                                    │
│  • Receives target position        │
│  • Plans trajectory                │
│  • Sends to hardware interface     │
└──────────┬─────────────────────────┘
           │
           │ Joint commands
           ↓
┌────────────────────────────────────┐
│   Gazebo Simulation                │
│   (Physics Engine)                 │
│                                    │
│  • Updates joint position          │
│  • Simulates dynamics              │
│  • Renders visualization           │
└────────────────────────────────────┘


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

```
┌─────────────────┐
│ gesture_detector│─┐
└─────────────────┘ │
                    │ publishes
                    ↓
              /elbow_position_controller/commands
                    │
                    │ subscribes
                    ↓
┌──────────────────────────────┐
│ elbow_position_controller    │
│ (ros2_control)               │
└──────────┬───────────────────┘
           │
           │ controls
           ↓
┌──────────────────────┐      ┌─────────────────────┐
│ joint_state_         │──────│ robot_state_        │
│ broadcaster          │      │ publisher           │
└──────────────────────┘      └─────────────────────┘
           │                           │
           │ publishes                 │ publishes
           ↓                           ↓
      /joint_states              /robot_description
```


## File Structure

```
cv_arm/
├── cv_arm/
│   ├── __init__.py
│   └── gesture_detector.py      ← Main gesture detection node
├── config/
│   └── elbow_controllers.yaml   ← Controller configuration
├── launch/
│   ├── spawn_elbow.launch.py    ← Basic robot spawn
│   └── gesture_control.launch.py ← Full system launch
├── model/
│   └── arm.urdf.xacro           ← Robot description
├── package.xml                  ← ROS2 package metadata
├── setup.py                     ← Python package setup
├── README.md                    ← Documentation
├── test_setup.py                ← Dependency checker
└── install_dependencies.sh      ← Installation script
```


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


## MediaPipe Hand Landmarks

```
       8 ← Index finger tip (used for control)
       │
       7
       │
       6
       │
   ┌───5
   │   │
   4───3───2───1───0 (wrist)
   │               │
  12              16
   │               │
  Thumb          Ring
```

Landmark indices:
- 0: Wrist
- 1-4: Thumb
- 5-8: Index finger (we use 8)
- 9-12: Middle finger
- 13-16: Ring finger
- 17-20: Pinky


## Coordinate Systems

### OpenCV Image Coordinates
```
(0,0) ────────────→ x (width)
  │
  │
  │
  ↓
  y (height)
```

### MediaPipe Normalized Coordinates
```
(0,0) ────────────→ x (0.0 to 1.0)
  │
  │
  │
  ↓
  y (0.0 to 1.0)
```

### Robot Joint Space
```
Elbow Joint Angle (radians)
0.1 rad (bent) ←→ 2.721 rad (extended)
```


## Control Loop Timing

```
Time (seconds)
0.0  │ Launch file starts
     │
3.0  │ Joint state broadcaster spawned
     │
4.0  │ Elbow controller spawned
     │
6.0  │ Gesture detector starts
     │
6.0+ │ ┌─────────────────────────┐
     │ │ Continuous control loop  │ @ 30 Hz
     │ │ (every ~33ms)           │
     │ └─────────────────────────┘
```

