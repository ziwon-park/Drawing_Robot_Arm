#!/usr/bin/env python3

import rospy
import numpy as np
from ur_control.arm import Arm
from ur_control import transformations
from math import pi, cos, sin
import tf

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA


#image processing
import cv2

#keyboard input
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


class RobotDrawer(object):
    def __init__(self, arm, marker_publisher, scale):
        self.arm = arm
        self.scale = scale
        self.initial_joint_positions = [1.57, -1.9, 1.4, -1.07, -1.57, 0]
        self.marker_publisher = marker_publisher


    def reset_to_initial_position(self):
        # 로봇 팔을 초기 위치로 이동
        self.arm.set_joint_positions(position=self.initial_joint_positions, wait=True, t=0.5)

    def draw_path(self, contours):
        if len(contours) == 0:
            return

        print("inside draw_path")

        marker_id = 0
        current_position = self.arm.end_effector()
        current_x, current_y, current_z = current_position[:3]
        print("current position is :", current_position[:3])

        first_point = contours[0][0][0]
        offset_x, offset_y = first_point

        # 로봇 팔의 현재 위치를 contour 시작점으로 설정
        for contour in contours:
            # Contour의 시작점과 끝점 처리
            for point_idx, point in enumerate(contour):
                # if point_idx >= 0 and point_idx <= len(contour):  # 시작점 또는 끝점
                x, y = point[0]
                # contour 좌표를 로봇 팔 좌표계로 변환
                x, y = (x * self.scale) + current_x, (y * self.scale) + current_y

                # 현재 위치와 목표 위치 출력
                print("target position is : ", [x, y, current_z])

                # 역기구학 솔루션 찾기 및 로봇 팔 움직임
                pose = [x, y, current_z] + list(current_position[3:])
                ik_solution = self.arm._solve_ik(pose)

                if ik_solution is not None:
                    self.arm.set_target_pose_flex(pose, t=1)
                    # 마커 발행
                    self.marker_publisher.publish_marker([x, y, current_z], frame_id="base_link", marker_id=marker_id)
                    marker_id += 1
                else:
                    rospy.logwarn("IK solution not found for pose: {}".format(pose))
                    break

                rospy.sleep(0.25)

                
    
    def run(self):
        while not rospy.is_shutdown():
            key = getch.getch()
            if key == 's':
                print("Resetting to initial position...")
                self.reset_to_initial_position()
                print("Robot arm has moved to the initial position")
            elif key == 'q':
                print("Quitting...")
                break

        # for contour in contours:
        #     # 이전 포인트 초기화
        #     prev_x, prev_y = None, None
        #     for point in contour:
        #         x, y = point[0]
        #         x, y = x * self.scale, y * self.scale  # 스케일 조정 및 좌표 변환

        #         if prev_x is not None and prev_y is not None:
        #             # 각 포인트 간의 방향 계산
        #             direction = np.array([x - prev_x, y - prev_y])
        #             angle = np.arctan2(direction[1], direction[0])

        #             # 쿼터니언으로 변환
        #             orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))
        #         else:
        #             # 첫 번째 포인트의 경우 이전 orientation 유지
        #             orientation = Quaternion(0, 0, 0, 1)  # 기본 orientation 설정

        #         # 로봇 팔 엔드 이펙터에 맞는 좌표계로 변환
        #         pose = [x, y, z_fixed] + [orientation.x, orientation.y, orientation.z, orientation.w]
        #         self.arm.set_target_pose_flex(pose, t=0.25)
        #         rospy.sleep(0.25)

        #         # 현재 포인트를 이전 포인트로 저장
        #         prev_x, prev_y = x, y

class CircleDrawer(object):
    def __init__(self, arm, marker_publisher, radius=0.1, resolution=100):
        self.arm = arm
        self.marker_publisher = marker_publisher
        self.radius = radius
        self.resolution = resolution

    def draw_circle(self):
        marker_id = 0
        center_pose = self.arm.end_effector()
        center_x = center_pose[0]
        center_y = center_pose[1]
        center_z = center_pose[2]

        for i in range(self.resolution):
            theta = 2 * pi * i / self.resolution
            x = center_x + self.radius * cos(theta)
            y = center_y + self.radius * sin(theta)
            z = center_z

            pose = [x, y, z] + list(center_pose[3:])
            print("pose is :", pose)
            self.arm.set_target_pose_flex(pose, t=0.25)
            self.marker_publisher.publish_marker([x, y, z], frame_id="base_link", marker_id=marker_id)
            marker_id += 1
            rospy.sleep(0.25)

class MarkerPublisher:
    def __init__(self):
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    def publish_marker(self, position, frame_id='base_link', namespace='robot_arm', marker_id=0):
        marker = Marker(
            type=Marker.SPHERE,
            id=marker_id,
            lifetime=rospy.Duration(0),
            pose=Pose(Point(*position), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.01, 0.01, 0.01),
            header=Header(frame_id=frame_id),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
            ns=namespace
        )
        self.marker_publisher.publish(marker)

def main():
    rospy.init_node("draw_image_with_ur3_markers", log_level=rospy.INFO)
    
    arm = Arm(ft_sensor=False, gripper=None,
              joint_names_prefix=None, robot_urdf="ur3",
              robot_urdf_package=None, ee_link=None)

    marker_publisher = MarkerPublisher()

    # 이미지 처리
    line_segments, hierarchy  = process_image("./src/cat2.png", resize_factor=1)

    # 이미지 그리기
    drawer = RobotDrawer(arm, marker_publisher, scale=0.001)  # 스케일 조정
    drawer.run()
    drawer.draw_path(line_segments)

    # circle_drawer = CircleDrawer(arm, marker_publisher, radius=0.1, resolution=100)
    # circle_drawer.draw_circle()

if __name__ == '__main__':
    main()
