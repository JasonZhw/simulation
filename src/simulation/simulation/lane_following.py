#!/usr/bin/env python3
"""
车道跟随节点 - 整合感知和控制
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Tuple, Optional, List


class LaneDetector:
    """车道检测类 - 使用你原有的滑动窗口方法"""
    
    def __init__(self, image_width: int = 848, image_height: int = 264,
                 vehicle_width: float = 0.6, vehicle_height: float = 0.4):
        self.image_width = image_width
        self.image_height = image_height
        self.vehicle_width = vehicle_width
        self.vehicle_height = vehicle_height
        
        self.px_to_m_w = vehicle_width / image_width
        self.px_to_m_h = vehicle_height / image_height
    
    def detect(self, binary_image: np.ndarray) -> Tuple[Optional[List], Optional[List], Optional[np.ndarray]]:
        if binary_image is None or binary_image.size == 0:
            return None, None, None
        
        if binary_image.dtype != np.uint8:
            img = (binary_image * 255).astype(np.uint8)
        else:
            img = binary_image.copy()
        
        left_lane, right_lane = self._detect_lanes_sliding_window(img)
        
        center_lane = None
        if left_lane is not None and right_lane is not None:
            center_lane = self._calculate_center_lane(left_lane, right_lane)
        
        return left_lane, right_lane, center_lane
    
    def get_lane_center_in_vehicle_frame(self, binary_image: np.ndarray) -> Optional[Tuple[float, float]]:
        _, _, center_lane = self.detect(binary_image)
        
        if center_lane is None or len(center_lane) == 0:
            return None
        
        bottom_point = center_lane[-1]
        x_img, y_img = bottom_point
        
        x_vehicle = (x_img - self.image_width / 2) * self.px_to_m_w
        y_vehicle = (self.image_height - y_img) * self.px_to_m_h
        
        return (x_vehicle, y_vehicle)
    
    def _detect_lanes_sliding_window(self, img: np.ndarray) -> Tuple[Optional[List], Optional[List]]:
        histogram = np.sum(img[img.shape[0]//2:, :], axis=0)
        
        midpoint = img.shape[1] // 2
        left_peak = np.argmax(histogram[:midpoint]) if np.max(histogram[:midpoint]) > 0 else None
        right_peak = np.argmax(histogram[midpoint:]) + midpoint if np.max(histogram[midpoint:]) > 0 else None
        
        if left_peak is None or right_peak is None:
            return None, None
        
        window_height = 30
        window_width = 50
        margin = 30
        
        left_lane = self._track_lane(img, left_peak, window_height, window_width, margin, "left")
        right_lane = self._track_lane(img, right_peak, window_height, window_width, margin, "right")
        
        return left_lane, right_lane
    
    def _track_lane(self, img: np.ndarray, start_x: int, window_height: int, 
                    window_width: int, margin: int, label: str) -> Optional[List]:
        lane_x = []
        lane_y = []
        current_x = start_x
        
        for y in range(img.shape[0] - 1, -1, -window_height):
            y_start = max(0, y - window_height)
            x_left = max(0, current_x - window_width // 2)
            x_right = min(img.shape[1], current_x + window_width // 2)
            
            window = img[y_start:y, x_left:x_right]
            
            if window.size == 0:
                continue
            
            white_indices = np.where(window > 127)
            
            if len(white_indices[0]) == 0:
                if len(lane_x) > 0:
                    current_x = int(np.mean(lane_x[-3:]))
                continue
            
            white_x_coords = white_indices[1] + x_left
            center_x = int(np.mean(white_x_coords))
            center_y = int((y + y_start) / 2)
            
            if len(lane_x) > 0:
                dx = center_x - lane_x[-1]
                dy = center_y - lane_y[-1]
                distance = np.sqrt(dx*dx + dy*dy)
                if distance > 100:
                    continue
            
            lane_x.append(center_x)
            lane_y.append(center_y)
            current_x = center_x
        
        if len(lane_x) < 5:
            return None
        
        return list(zip(lane_x[::-1], lane_y[::-1]))
    
    def _calculate_center_lane(self, left_lane: List, right_lane: List) -> Optional[np.ndarray]:
        if not left_lane or not right_lane:
            return None
        
        left_array = np.array(left_lane)
        right_array = np.array(right_lane)
        
        y_min = max(left_array[:, 1].min(), right_array[:, 1].min())
        y_max = min(left_array[:, 1].max(), right_array[:, 1].max())
        
        if y_max <= y_min:
            return None
        
        y_common = np.arange(int(y_min), int(y_max) + 1)
        
        try:
            left_x_interp = np.interp(y_common, left_array[:, 1], left_array[:, 0])
            right_x_interp = np.interp(y_common, right_array[:, 1], right_array[:, 0])
        except:
            return None
        
        center_x = (left_x_interp + right_x_interp) / 2
        center_lane = np.column_stack([center_x, y_common])
        return center_lane


class BEVProcessor:
    """鸟瞰图处理 - 使用你原有的代码"""
    
    def __init__(self):
        self.USE_FIXED_POINTS = True
        self.SRC_POINTS_PX = np.float32([
            [297, 288],
            [563, 288],
            [748, 365],
            [120, 365],
        ])
        self.OUT_H_FRAC = 0.55
        self.PAD_X = 250
        self.FIXED_THRESH = 140
        self.BLUR_KSIZE = (5, 5)
    
    def process(self, image_bgr):
        h, w = image_bgr.shape[:2]
        src = self.SRC_POINTS_PX.copy()
        
        out_w = w
        out_h = int(self.OUT_H_FRAC * h)
        dst = np.float32([
            [self.PAD_X, 0],
            [out_w - self.PAD_X, 0],
            [out_w - self.PAD_X, out_h],
            [self.PAD_X, out_h],
        ])
        
        H = cv2.getPerspectiveTransform(src, dst)
        bev = cv2.warpPerspective(image_bgr, H, (out_w, out_h),
                                   flags=cv2.INTER_LINEAR, 
                                   borderMode=cv2.BORDER_CONSTANT, 
                                   borderValue=(0, 0, 0))
        
        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, self.BLUR_KSIZE, 0)
        _, mask = cv2.threshold(gray, self.FIXED_THRESH, 255, cv2.THRESH_BINARY)
        
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask, bev


class LaneFollowingNode(Node):
    """车道跟随主节点"""
    
    def __init__(self):
        super().__init__('lane_following_node')
        
        # 参数
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('kd', 0.5)
        
        self.max_speed = self.get_parameter('max_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        
        # 初始化
        self.bridge = CvBridge()
        self.bev_processor = BEVProcessor()
        self.lane_detector = LaneDetector(image_width=848, image_height=264)
        
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        
        # 订阅和发布
        self.image_sub = self.create_subscription(
            Image, '/front_camera/image_raw', self.image_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Float32, '/lane_error', 10)
        self.bev_pub = self.create_publisher(Image, '/bev_image', 10)
        
        self.get_logger().info('Lane following node started')
    
    def image_callback(self, msg):
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # BEV转换
            binary_mask, bev_image = self.bev_processor.process(cv_image)
            
            # 发布BEV图像用于可视化
            bev_msg = self.bridge.cv2_to_imgmsg(bev_image, 'bgr8')
            self.bev_pub.publish(bev_msg)
            
            # 车道检测
            center_pos = self.lane_detector.get_lane_center_in_vehicle_frame(binary_mask)
            
            if center_pos is not None:
                x_vehicle, y_vehicle = center_pos
                
                # 计算横向误差（相对于车辆中心）
                lateral_error = x_vehicle
                
                # PD控制计算角速度
                current_time = self.get_clock().now()
                dt = (current_time - self.prev_time).nanoseconds / 1e9
                
                if dt > 0:
                    error_derivative = (lateral_error - self.prev_error) / dt
                else:
                    error_derivative = 0.0
                
                angular_z = -(self.kp * lateral_error + self.kd * error_derivative)
                angular_z = np.clip(angular_z, -2.0, 2.0)
                
                # 根据转弯程度调整速度
                speed_factor = 1.0 - min(abs(angular_z) / 2.0, 0.5)
                linear_x = self.max_speed * speed_factor
                
                # 发布控制指令
                cmd = Twist()
                cmd.linear.x = linear_x
                cmd.angular.z = angular_z
                self.cmd_pub.publish(cmd)
                
                # 发布误差
                error_msg = Float32()
                error_msg.data = lateral_error
                self.error_pub.publish(error_msg)
                
                # 更新状态
                self.prev_error = lateral_error
                self.prev_time = current_time
                
                self.get_logger().info(
                    f'Error: {lateral_error:.3f}m, Speed: {linear_x:.2f}, Angular: {angular_z:.2f}',
                    throttle_duration_sec=0.5)
            else:
                # 未检测到车道线，停车
                cmd = Twist()
                self.cmd_pub.publish(cmd)
                self.get_logger().warn('No lane detected, stopping', throttle_duration_sec=1.0)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
