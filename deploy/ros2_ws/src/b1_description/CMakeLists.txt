cmake_minimum_required(VERSION 3.5)
project(b1_description)
find_package(ament_cmake REQUIRED)

# Install launch files.
install(DIRECTORY
  launch robots meshes rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()