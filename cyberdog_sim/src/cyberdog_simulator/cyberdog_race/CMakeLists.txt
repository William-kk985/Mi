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
option(USE_TEST_REAL_STAGE4 "Real stage4 test version (废弃: 2026-09-02 改为运行期参数 stage4_impl, 见 race.launch.py)" OFF)
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

# CyberDog2 真机 motion 接口（protocol 包仅真机 NX 有，仿真环境没有 → QUIET 保护）
find_package(protocol QUIET)
if(protocol_FOUND)
  message(STATUS "[REAL] protocol 包可用 → 启用真机 motion_servo_cmd 支持")
endif()

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
  set(STAGE1_SRC src/test/virtual/src/stage1_test.cpp)
  add_compile_definitions(USE_TEST_STAGE1)
  message(STATUS "[TEST] Stage1 → src/test/virtual/src/stage1_test.cpp")
endif()
if(USE_TEST_ALL OR USE_TEST_STAGE2)
  set(STAGE2_SRC src/test/virtual/src/stage2_test.cpp)
  add_compile_definitions(USE_TEST_STAGE2)
  message(STATUS "[TEST] Stage2 → src/test/virtual/src/stage2_test.cpp")
endif()
if(USE_TEST_ALL OR USE_TEST_STAGE3)
  set(STAGE3_SRC src/test/virtual/src/stage3_test.cpp)
  add_compile_definitions(USE_TEST_STAGE3)
  message(STATUS "[TEST] Stage3 → src/test/virtual/src/stage3_test.cpp")
endif()
if(USE_TEST_ALL OR USE_TEST_STAGE4)
  set(STAGE4_SRC src/test/virtual/src/stage4_test.cpp)
  add_compile_definitions(USE_TEST_STAGE4)
  message(STATUS "[TEST] Stage4 → src/test/virtual/src/stage4_test.cpp")
endif()
if(USE_TEST_ALL OR USE_TEST_STAGE5)
  set(STAGE5_SRC src/test/virtual/src/stage5_test.cpp)
  add_compile_definitions(USE_TEST_STAGE5)
  message(STATUS "[TEST] Stage5 → src/test/virtual/src/stage5_test.cpp")
endif()
if(USE_TEST_ALL OR USE_TEST_STAGE6)
  set(STAGE6_SRC src/test/virtual/src/stage6_test.cpp)
  add_compile_definitions(USE_TEST_STAGE6)
  message(STATUS "[TEST] Stage6 → src/test/virtual/src/stage6_test.cpp")
endif()

set(SOURCES
  src/main.cpp
  src/race_controller.cpp
  src/test/function/src/behavior_test.cpp
  src/motion_ctrl.cpp
  src/vision/virtual/lane_detector.cpp
  src/vision/virtual/ball_detector.cpp

  src/vision/real/yolo_detector.cpp
  src/vision/real/lidar_locator.cpp
  src/utils/web_streamer.cpp
  src/utils/llm_helper.cpp
  ${STAGE1_SRC}
  ${STAGE2_SRC}
  ${STAGE3_SRC}
  ${STAGE4_SRC}
  ${STAGE5_SRC}
  ${STAGE6_SRC}
  # ── 真机赛段（有实现后取消注释） ──
  src/stages/real/stage1_real.cpp
  src/stages/real/stage2_real.cpp
  src/stages/real/stage3_real.cpp
  src/stages/real/stage4_real.cpp
  src/stages/real/stage5_real.cpp
  src/stages/real/stage6_real.cpp
)

# ── 真机赛段测试替换（追加到 SOURCES） ──
macro(add_real_test N)
  if(USE_TEST_REAL_ALL OR USE_TEST_REAL_STAGE${N})
    list(APPEND SOURCES src/test/real/src/stage${N}_real_test.cpp)
    add_compile_definitions(USE_TEST_REAL_STAGE${N})
    message(STATUS "[TEST] Real Stage${N} → src/test/real/src/stage${N}_real_test.cpp")
  endif()
endmacro()
# ── Stage4 test 版：无条件编入 (2026-09-02 重构: 全编 + 运行期 stage4_impl 参数选) ──
#   不再用 -DUSE_TEST_REAL_STAGE4 编译期开关; launch 传 stage4_impl:=test/test2 即选中
list(APPEND SOURCES src/test/real/src/stage4_real_test.cpp)
list(APPEND SOURCES src/test/real/src/stage4_real_test2.cpp)

add_real_test(1)
add_real_test(2)
add_real_test(3)
# add_real_test(4)  ← 已由上方无条件编入取代 (2026-09-02)
add_real_test(5)
add_real_test(6)

# ── 伙伴连通域寻线算法实验 (2026-08-14) ──
# USE_TEST_REAL_STAGE3 时整体替换 lane_detector.cpp → 测试版 v2
#   v2 类名/接口与正式版同构, race_controller 通过 USE_TEST_LANE_V2 条件include
if(USE_TEST_REAL_STAGE3 OR USE_TEST_REAL_ALL)
  list(REMOVE_ITEM SOURCES src/vision/virtual/lane_detector.cpp)
  list(APPEND SOURCES src/test/real/src/lane_detector_v2.cpp)
  add_compile_definitions(USE_TEST_LANE_V2)
  message(STATUS "[TEST] LaneDetector → 伙伴连通域版 (src/test/real/src/lane_detector_v2.cpp)")
endif()

add_executable(race_controller ${SOURCES})
ament_target_dependencies(race_controller rclcpp sensor_msgs nav_msgs cyberdog_msg cv_bridge)
if(protocol_FOUND)
  ament_target_dependencies(race_controller protocol)
endif()
rosidl_target_interfaces(race_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(race_controller lcm ${OpenCV_LIBS})

# LLM_MODE_API needs libcurl
find_package(CURL QUIET)
if(CURL_FOUND)
  target_include_directories(race_controller PRIVATE ${CURL_INCLUDE_DIRS})
  target_link_libraries(race_controller ${CURL_LIBRARIES})
endif()

install(TARGETS race_controller DESTINATION lib/${PROJECT_NAME})

# ── launch 文件 (2026-09-02 重构: 运行期版本选择入口) ──
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ── center_cam: 官方 camera_api 桥接节点 (2026-08-14) ──
#   把 cam_id 0/1/2/3 的相机(bottom/center 模组)发成 ROS topic
#   仅真机: camera_api 库只在 NX /opt/ros2/cyberdog 有
find_library(CAMERA_API_LIB camera_api PATHS /opt/ros2/cyberdog/lib NO_DEFAULT_PATH)
if(CAMERA_API_LIB)
  include_directories(/opt/ros2/cyberdog/include)
  add_executable(center_cam src/vision/real/center_cam.cpp)
  ament_target_dependencies(center_cam rclcpp sensor_msgs cv_bridge)
  target_link_libraries(center_cam ${CAMERA_API_LIB} ${OpenCV_LIBS})
  install(TARGETS center_cam DESTINATION lib/${PROJECT_NAME})
  message(STATUS "[CAM] center_cam 已启用 (libcamera_api)")
else()
  message(STATUS "[CAM] 未找到 camera_api 库, 跳过 center_cam")
endif()

ament_package()
