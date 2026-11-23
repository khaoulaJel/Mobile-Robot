cmake_minimum_required(VERSION 3.8)
project(lab3_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/odom_to_pose.py DESTINATION lib/${PROJECT_NAME})

ament_package()
