cmake_minimum_required(VERSION 3.8)
project(cyberdog_race)

# ═══ 测试版替换开关 ═══
# 编译时传参：colcon build --cmake-args -DUSE_TEST_STAGE1=ON
# config.hpp 中对应宏定义（如 USE_TEST_STAGE1）在 add_compile_definitions 中同步
option(USE_TEST_STAGE1 "Replace stage1.cpp with stage1_test.cpp" OFF)
option(USE_TEST_STAGE2 "Replace stage2.cpp with stage2_test.cpp" OFF)
option(USE_TEST_STAGE3 "Replace stage3.cpp with stage3_test.cpp" OFF)
option(USE_TEST_STAGE4 "Replace stage4.cpp with stage4_test.cpp" OFF)
option(USE_TEST_STAGE5 "Replace stage5.cpp with stage5_test.cpp" OFF)
option(USE_TEST_STAGE6 "Replace stage6.cpp with stage6_test.cpp" OFF)
option(USE_TEST_ALL   "Replace ALL stages with _test.cpp versions" OFF)
option(USE_TEST_REAL_STAGE1 "Real stage1 test version" OFF)
option(USE_TEST_REAL_STAGE2 "Real stage2 test version" OFF)
option(USE_TEST_REAL_STAGE3 "Real stage3 test version" OFF)
option(USE_TEST_REAL_STAGE4 "Real stage4 test version" OFF)
option(USE_TEST_REAL_STAGE5 "Real stage5 test version" OFF)
option(USE_TEST_REAL_STAGE6 "Real stage6 test version" OFF)
option(USE_TEST_REAL_ALL   "All real stages test version" OFF)

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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LLMAsk.srv"
)

include_directories(include
  src/test/function/inc
  src/test/real/inc
  src/test/virtual/inc
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../cyberdog_gazebo/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../cyberdog_visual/lcm_type
  ${CMAKE_CURRENT_SOURCE_DIR}/../../cyberdog_locomotion/common
  ${CMAKE_CURRENT_SOURCE_DIR}/../../cyberdog_locomotion/common/lcm_type/lcm
)

# ═══ 赛段源文件选择（测试版替换） ═══
# 宏：auto_init LCM 重建端口
set(STAGE1_SRC src/stages/virtual/stage1.cpp)
set(STAGE2_SRC src/stages/virtual/stage2.cpp)
set(STAGE3_SRC src/stages/virtual/stage3.cpp)
set(STAGE4_SRC src/stages/virtual/stage4.cpp)
set(STAGE5_SRC src/stages/virtual/stage5.cpp)
set(STAGE6_SRC src/stages/virtual/stage6.cpp)

if(USE_TEST_ALL OR USE_TEST_STAGE1)
  set(STAGE1_SRC src/stages/virtual/stage1_test.cpp)
  add_compile_definitions(USE_TEST_STAGE1)
  message(STATUS "[TEST] Stage1 → stage1_test.cpp")
endif()
if(USE_TEST_ALL OR USE_TEST_STAGE2)
  set(STAGE2_SRC src/stages/virtual/stage2_test.cpp)
  add_compile_definitions(USE_TEST_STAGE2)
  message(STATUS "[TEST] Stage2 → stage2_test.cpp")
endif()
if(USE_TEST_ALL OR USE_TEST_STAGE3)
  set(STAGE3_SRC src/stages/virtual/stage3_test.cpp)
  add_compile_definitions(USE_TEST_STAGE3)
  message(STATUS "[TEST] Stage3 → stage3_test.cpp")
endif()
if(USE_TEST_ALL OR USE_TEST_STAGE4)
  set(STAGE4_SRC src/stages/virtual/stage4_test.cpp)
  add_compile_definitions(USE_TEST_STAGE4)
  message(STATUS "[TEST] Stage4 → stage4_test.cpp")
endif()
if(USE_TEST_ALL OR USE_TEST_STAGE5)
  set(STAGE5_SRC src/stages/virtual/stage5_test.cpp)
  add_compile_definitions(USE_TEST_STAGE5)
  message(STATUS "[TEST] Stage5 → stage5_test.cpp")
endif()
if(USE_TEST_ALL OR USE_TEST_STAGE6)
  set(STAGE6_SRC src/stages/virtual/stage6_test.cpp)
  add_compile_definitions(USE_TEST_STAGE6)
  message(STATUS "[TEST] Stage6 → stage6_test.cpp")
endif()

set(SOURCES
  src/main.cpp
  src/race_controller.cpp
  src/test/function/src/behavior_test.cpp
  src/motion_ctrl.cpp
  src/vision/virtual/lane_detector.cpp
  src/vision/virtual/ball_detector.cpp
  src/vision/virtual/stage4_detector.cpp
  src/utils/web_streamer.cpp
  src/llm_helper.cpp
  ${STAGE1_SRC}
  ${STAGE2_SRC}
  ${STAGE3_SRC}
  ${STAGE4_SRC}
  ${STAGE5_SRC}
  ${STAGE6_SRC}
  # ── 真机赛段（TODO: 有实现后取消注释） ──
  # src/stages/real/stage1_real.cpp
  # src/stages/real/stage2_real.cpp
  # src/stages/real/stage3_real.cpp
  # src/stages/real/stage4_real.cpp
  # src/stages/real/stage5_real.cpp
  # src/stages/real/stage6_real.cpp
)

# ── 真机赛段测试替换（追加到 SOURCES） ──
macro(add_real_test N)
  if(USE_TEST_REAL_ALL OR USE_TEST_REAL_STAGE${N})
    list(APPEND SOURCES src/test/real/src/stage${N}_real_test.cpp)
    add_compile_definitions(USE_TEST_REAL_STAGE${N})
    message(STATUS "[TEST] Real Stage${N} → src/test/real/src/stage${N}_real_test.cpp")
  endif()
endmacro()
add_real_test(1)
add_real_test(2)
add_real_test(3)
add_real_test(4)
add_real_test(5)
add_real_test(6)

add_executable(race_controller ${SOURCES})
ament_target_dependencies(race_controller rclcpp sensor_msgs nav_msgs cyberdog_msg cv_bridge)
rosidl_target_interfaces(race_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(race_controller lcm ${OpenCV_LIBS})

# LLM_MODE_API needs libcurl
find_package(CURL QUIET)
if(CURL_FOUND)
  target_include_directories(race_controller PRIVATE ${CURL_INCLUDE_DIRS})
  target_link_libraries(race_controller ${CURL_LIBRARIES})
endif()

install(TARGETS race_controller DESTINATION lib/${PROJECT_NAME})

ament_package()
