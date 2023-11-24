#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi, cos, sin

class CircleDrawer(object):
    def __init__(self):
        super(CircleDrawer, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('circle_drawer', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "ur3"  
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def draw_circle(self, radius=0.1, resolution=100, scale=1):
        waypoints = []

        # 원의 중심을 현재 위치로 설정
        center_pose = self.move_group.get_current_pose().pose
        center_x = center_pose.position.x
        center_y = center_pose.position.y
        center_z = center_pose.position.z

        # 원의 웨이포인트 생성
        for i in range(resolution):
            theta = 2 * pi * i / resolution
            x = center_x + radius * cos(theta)
            y = center_y + radius * sin(theta)
            z = center_z  # 원은 xy 평면에 있음

            pose = geometry_msgs.msg.Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation = center_pose.orientation

            waypoints.append(pose)

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        return plan, fraction

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

def main():
    try:
        circle_drawer = CircleDrawer()

        # 원 그리기
        rospy.sleep(2)
        circle_plan, _ = circle_drawer.draw_circle()
        circle_drawer.execute_plan(circle_plan)

        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
