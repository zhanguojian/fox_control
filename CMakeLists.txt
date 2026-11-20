cmake_minimum_required(VERSION 3.8)
project(akm_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg_ros/AKMActuatorState.msg"
  "msg_ros/AKMLightCmd.msg"
  "msg_ros/AKMLightState.msg"
  "msg_ros/AKMRCState.msg"
  "msg_ros/AKMStatus.msg"
  DEPENDENCIES std_msgs
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_executable(fox_control
  src/akm_base_node.cpp
  src/akm_robot.cpp
  src/async/async_can.cpp
  src/protocols/msg_praser.cpp
)

target_include_directories(fox_control PRIVATE
  include
)

target_link_libraries(fox_control
  Boost::system
  Threads::Threads
  yaml-cpp
)

ament_target_dependencies(fox_control
  geometry_msgs
  rclcpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(fox_control "${cpp_typesupport_target}")

install(TARGETS fox_control
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
