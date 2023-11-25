#!/usr/bin/env python3

import rospy
import numpy as np
from ur_control.arm import Arm
from ur_control import transformations
from math import pi, cos, sin

class CircleDrawer(object):
    def __init__(self, arm, radius=0.1, resolution=100):
        self.arm = arm
        self.radius = radius
        self.resolution = resolution

    def draw_circle(self):
        # 원의 중심을 현재 엔드 이펙터 위치로 설정
        center_pose = self.arm.end_effector()
        center_x = center_pose[0]
        center_y = center_pose[1]
        center_z = center_pose[2]

        # 원 경로 계산
        for i in range(self.resolution):
            theta = 2 * pi * i / self.resolution
            x = center_x + self.radius * cos(theta)
            y = center_y + self.radius * sin(theta)
            z = center_z

            # 새로운 위치로 엔드 이펙터 이동
            pose = [x, y, z] + list(center_pose[3:])
            self.arm.set_target_pose_flex(pose, t=0.25)
            rospy.sleep(0.25)

def main():
    rospy.init_node("draw_circle_with_ur3", log_level=rospy.INFO)

    # UR3 로봇 팔 초기화
    arm = Arm(ft_sensor=False, gripper=None,
              joint_names_prefix=None, robot_urdf="ur3e",
              robot_urdf_package=None, ee_link=None)

    # 원 그리기
    circle_drawer = CircleDrawer(arm, radius=0.1, resolution=100)
    circle_drawer.draw_circle()

    print("Circle drawing complete.")

if __name__ == '__main__':
    main()
