cmake_minimum_required(VERSION 3.0.2)
project(interactive_stl_marker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  visualization_msgs
  )


catkin_python_setup()

generate_messages(
  DEPENDENCIES
  visualization_msgs
)

catkin_package()
