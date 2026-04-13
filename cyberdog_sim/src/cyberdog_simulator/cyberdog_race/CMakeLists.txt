cmake_minimum_required(VERSION 3.8)
project(cyberdog_race)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cyberdog_msg REQUIRED)
find_package(lcm REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../cyberdog_gazebo/include
)

set(SOURCES
  src/main.cpp
  src/motion_ctrl.cpp
  src/vision/lane_detector.cpp
  src/vision/ball_detector.cpp
  src/stages/stage1.cpp
  src/stages/stage2.cpp
  src/stages/stage3.cpp
  src/stages/stage4.cpp
  src/stages/stage5.cpp
  src/stages/stage6.cpp
)

add_executable(race_controller ${SOURCES})
ament_target_dependencies(race_controller rclcpp sensor_msgs nav_msgs cyberdog_msg cv_bridge)
target_link_libraries(race_controller lcm ${OpenCV_LIBS})

install(TARGETS race_controller DESTINATION lib/${PROJECT_NAME})

ament_package()
