#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from ur_control.arm import Arm  # 이 부분은 ROS2에 맞게 수정 필요
from ur_control import transformations  # 이 부분은 ROS2에 맞게 수정 필요
from math import pi, cos, sin
import tf

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA

import cv2
import getch



def process_image(image_path, resize_factor=1.0):
    # 이미지 읽기
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # 이미지 리사이징
    if resize_factor != 1.0:
        new_size = (int(image.shape[1] * resize_factor), int(image.shape[0] * resize_factor))
        image = cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)

    # 선명도 향상을 위한 처리
    kernel_sharpening = np.array([[-1, -1, -1],
                                  [-1, 9, -1],
                                  [-1, -1, -1]])
    image = cv2.filter2D(image, -1, kernel_sharpening)

    # 이진화를 통한 라인 추출
    _, binary = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)

    # 스켈레톤화
    skeleton = cv2.ximgproc.thinning(binary)

    # 라인 추출을 위한 이미지 처리 및 계층 정보 획득
    contours, hierarchy = cv2.findContours(skeleton, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 각 선분의 픽셀 좌표 추출
    line_segments = [cv2.approxPolyDP(contour, 1, False) for contour in contours]

    return line_segments, hierarchy

class RobotDrawer(Node):
    def __init__(self):
        super().__init__('robot_drawer')
        self.arm = Arm(ft_sensor=False, gripper=None, joint_names_prefix=None, robot_urdf="ur3", robot_urdf_package=None, ee_link=None)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.scale = self.declare_parameter('scale', 0.001).get_parameter_value().double_value
        self.initial_joint_positions = [1.57, -1.9, 1.4, -1.07, -1.57, 0]


def main(args=None):
    rclpy.init(args=args)

    robot_drawer = RobotDrawer()

    rclpy.spin(robot_drawer)

    robot_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
