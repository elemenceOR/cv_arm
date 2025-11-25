#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import cv2
import mediapipe as mp
import numpy as np
import sys

class GestureDetector(Node):
    """
    Node that detects hand gestures using MediaPipe and publishes elbow angle commands.
    """
    
    def __init__(self):
        super().__init__('gesture_detector')
        
        self.declare_parameter('camera_source', 'http://192.168.0.249:8080/video')
        
        source_param = self.get_parameter('camera_source').get_parameter_value().string_value
        
        try:
            self.camera_source = int(source_param)
        except ValueError:
            self.camera_source = source_param

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/elbow_position_controller/commands',
            10
        )
        
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Initialize Camera
        self.get_logger().info(f'Connecting to camera source: {self.camera_source}')
        self.cap = cv2.VideoCapture(self.camera_source)
        
        # Joint limits
        self.joint_min = 0.1
        self.joint_max = 2.721
        self.current_angle = self.joint_min
        self.smoothing = 0.2
        
        self.timer = self.create_timer(0.033, self.process_frame)
        
        self.get_logger().info('Gesture Detector Started')
        self.get_logger().info('Use --ros-args -p camera_source:="http://..." to change camera')

    def process_frame(self):
        if not self.cap.isOpened():
            self.get_logger().warn(f'Cannot access camera at {self.camera_source}')
            # Try to reconnect occasionally
            self.cap.open(self.camera_source)
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame. (Is phone screen off?)')
            return
        
        # Resize large phone images to speed up processing (optional but recommended)
        frame = cv2.resize(frame, (640, 480))
        
        # Flip (mirror effect)
        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        results = self.hands.process(rgb_frame)
        
        target_angle = self.current_angle
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                index_tip = hand_landmarks.landmark[8]
                clamped_y = max(0.0, min(1.0, index_tip.y))
                
                # Logic: Top (0.0) = Min Angle, Bottom (1.0) = Max Angle
                target_angle = self.joint_min + clamped_y * (self.joint_max - self.joint_min)
                
                # Visuals
                cv2.rectangle(frame, (w-30, int(clamped_y*h)), (w-10, h), (0, 255, 0), -1)
                cv2.rectangle(frame, (w-30, 0), (w-10, h), (255, 255, 255), 2)

        self.current_angle = (self.smoothing * target_angle + (1 - self.smoothing) * self.current_angle)
        
        msg = Float64MultiArray()
        msg.data = [self.current_angle]
        self.publisher.publish(msg)
        
        cv2.imshow('Gesture Control', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GestureDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()