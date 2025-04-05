cmake_minimum_required(VERSION 3.5)
project(gripper_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
