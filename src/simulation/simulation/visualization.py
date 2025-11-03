#!/usr/bin/env python3
"""
可视化节点 - 可选，用于调试
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisualizationNode(Node):
    
    def __init__(self):
        super().__init__('visualization_node')
        
        self.bridge = CvBridge()
        self.latest_bev = None
        self.latest_error = 0.0
        
        # 订阅
        self.bev_sub = self.create_subscription(
            Image, '/bev_image', self.bev_callback, 10)
        self.error_sub = self.create_subscription(
            Float32, '/lane_error', self.error_callback, 10)
        
        # 定时器显示
        self.timer = self.create_timer(0.1, self.display_callback)
        
        self.get_logger().info('Visualization node started')
    
    def bev_callback(self, msg):
        try:
            self.latest_bev = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting BEV image: {str(e)}')
    
    def error_callback(self, msg):
        self.latest_error = msg.data
    
    def display_callback(self):
        if self.latest_bev is not None:
            display_img = self.latest_bev.copy()
            
            # 绘制中心线
            h, w = display_img.shape[:2]
            center_x = w // 2
            cv2.line(display_img, (center_x, 0), (center_x, h), (0, 255, 0), 2)
            
            # 显示误差
            error_text = f'Error: {self.latest_error:.3f}m'
            cv2.putText(display_img, error_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 显示误差指示器
            indicator_x = int(center_x + self.latest_error * 200)
            indicator_x = np.clip(indicator_x, 0, w-1)
            cv2.circle(display_img, (indicator_x, h-20), 10, (0, 0, 255), -1)
            
            cv2.imshow('Lane Following Visualization', display_img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
