#!/usr/bin/env python3
"""
Test script to check if all dependencies are installed correctly
"""

import sys

def test_imports():
    """Test if all required packages can be imported"""
    tests = []
    
    # Test OpenCV
    try:
        import cv2
        print(f"✓ OpenCV installed (version {cv2.__version__})")
        tests.append(True)
    except ImportError as e:
        print(f"✗ OpenCV not found: {e}")
        tests.append(False)
    
    # Test MediaPipe
    try:
        import mediapipe as mp
        print(f"✓ MediaPipe installed (version {mp.__version__})")
        tests.append(True)
    except ImportError as e:
        print(f"✗ MediaPipe not found: {e}")
        tests.append(False)
    
    # Test NumPy
    try:
        import numpy as np
        print(f"✓ NumPy installed (version {np.__version__})")
        tests.append(True)
    except ImportError as e:
        print(f"✗ NumPy not found: {e}")
        tests.append(False)
    
    # Test ROS2
    try:
        import rclpy
        print(f"✓ ROS2 rclpy installed")
        tests.append(True)
    except ImportError as e:
        print(f"✗ ROS2 rclpy not found: {e}")
        tests.append(False)
    
    # Test std_msgs
    try:
        from std_msgs.msg import Float64MultiArray
        print(f"✓ std_msgs installed")
        tests.append(True)
    except ImportError as e:
        print(f"✗ std_msgs not found: {e}")
        tests.append(False)
    
    return all(tests)

def test_camera():
    """Test if camera is accessible"""
    try:
        import cv2
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret:
                print(f"✓ Camera is accessible (resolution: {frame.shape[1]}x{frame.shape[0]})")
                return True
            else:
                print("✗ Camera opened but cannot read frames")
                return False
        else:
            print("✗ Cannot open camera (device index 0)")
            print("  Try: ls /dev/video*")
            return False
    except Exception as e:
        print(f"✗ Camera test failed: {e}")
        return False

def main():
    print("=" * 50)
    print("CV Arm - Dependency Check")
    print("=" * 50)
    print()
    
    print("Testing Python packages...")
    print("-" * 50)
    deps_ok = test_imports()
    print()
    
    print("Testing camera access...")
    print("-" * 50)
    camera_ok = test_camera()
    print()
    
    print("=" * 50)
    if deps_ok and camera_ok:
        print("✓ All tests passed! Ready to run gesture control.")
        print()
        print("Run with: ros2 launch cv_arm gesture_control.launch.py")
        return 0
    else:
        print("✗ Some tests failed. Please install missing dependencies.")
        print()
        if not deps_ok:
            print("Install dependencies with:")
            print("  sudo apt-get install python3-pip python3-opencv")
            print("  pip3 install mediapipe")
        if not camera_ok:
            print()
            print("Check camera:")
            print("  ls /dev/video*")
        return 1

if __name__ == '__main__':
    sys.exit(main())
