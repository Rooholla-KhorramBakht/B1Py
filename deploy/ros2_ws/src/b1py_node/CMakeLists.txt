cmake_minimum_required(VERSION 3.5)
project(b1py_node)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(unitree_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
  link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/x86_64)
else()
  set(ARCH arm64)
  link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/aarch64)
endif()

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)


set(CMAKE_CXX_FLAGS "-O3")

include_directories(
    include
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
    # ${CMAKE_SOURCE_DIR}/../lcm_types
)

add_executable(highlevel src/highlevel.cpp)
target_link_libraries(highlevel ${EXTRA_LIBS})
ament_target_dependencies(highlevel rclcpp unitree_msgs sensor_msgs geometry_msgs)

install(TARGETS
    highlevel
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()




