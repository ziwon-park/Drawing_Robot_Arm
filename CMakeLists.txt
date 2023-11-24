cmake_minimum_required(VERSION 2.8.3)
project(ur3_moveit_pkg)

## 컴파일러 옵션 추가
add_compile_options(-std=c++11)

## 필요한 ROS 패키지 명시
find_package(catkin REQUIRED COMPONENTS
  rospy
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
)

## catkin 특정 구성 요소 설정
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur3_moveit_pkg
#  CATKIN_DEPENDS rospy moveit_core moveit_ros_planning_interface
#  DEPENDS system_lib
)

## Include 디렉토리 명시
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## catkin_python_setup()을 호출하여 Python 패키지를 설정합니다
catkin_python_setup()
