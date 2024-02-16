cmake_minimum_required(VERSION 3.10.2)
project(franka_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()