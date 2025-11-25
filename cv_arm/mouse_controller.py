#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np


class MouseController(Node):
    def __init__(self):
        super().__init__('mouse_controller')
        
        self.publisher = self.create_publisher(Float64MultiArray, '/elbow_position_controller/commands', 10)
        self.joint_min = 0.1
        self.joint_max = 2.721
        self.current_angle = self.joint_min
        self.smoothing = 0.3
        
        self.window_name = 'Mouse Control'
        cv2.namedWindow(self.window_name)
        self.img_height = 480
        self.img_width = 640
        self.mouse_y = self.img_height // 2
        
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info('Mouse Controller Started!')
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_y = y
    
    def process_frame(self):
        image = np.zeros((self.img_height, self.img_width, 3), np.uint8)
        normalized_y = 1.0 - (self.mouse_y / self.img_height)
        normalized_y = max(0.0, min(1.0, normalized_y))
        target_angle = self.joint_min + normalized_y * (self.joint_max - self.joint_min)
        self.current_angle = (self.smoothing * target_angle + (1 - self.smoothing) * self.current_angle)
        
        msg = Float64MultiArray()
        msg.data = [self.current_angle]
        self.publisher.publish(msg)
        
        cv2.line(image, (self.img_width//2, 0), (self.img_width//2, self.img_height), (50, 50, 50), 2)
        cv2.circle(image, (self.img_width//2, self.mouse_y), 20, (0, 255, 0), -1)
        angle_height = int((1.0 - normalized_y) * self.img_height)
        cv2.rectangle(image, (20, angle_height), (60, self.img_height), (0, 165, 255), -1)
        
        cv2.putText(image, 'Mouse Control', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(image, f'Angle: {self.current_angle:.3f} rad', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(image, f'Degrees: {np.degrees(self.current_angle):.1f}', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(image, 'Move mouse UP/DOWN', (10, self.img_height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, 'Press Q to quit', (10, self.img_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        
        cv2.imshow(self.window_name, image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MouseController()
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
