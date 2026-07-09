#pragma once

// ============================================================
// 日志/可视化开关
// 正式比赛前全部注释掉，重新 build = 零额外开销
// ============================================================

// #define DEBUG_VISION    // 视觉可视化（cv::imshow，黄线边界，中心线，球检测）
// #define DEBUG_MOTION    // 运动控制日志（速度指令输出）
// #define DEBUG_SENSOR    // 传感器数据日志（IMU/里程计/Lidar）
// #define DEBUG_STAGE     // 绿色终端打印（受 DEBUG_STAGE 控制）

#ifdef DEBUG_STAGE
#define LOG_STAGE_GREEN(tag, msg) fprintf(stderr, "\033[1;34m[" tag "] " msg "\033[0m\n")
#define LOG_STAGE_GREENF(tag, fmt, ...) fprintf(stderr, "\033[1;34m[" tag "] " fmt "\033[0m\n", ##__VA_ARGS__)
#else
#define LOG_STAGE_GREEN(tag, msg)
#define LOG_STAGE_GREENF(tag, fmt, ...)
#endif

// ============================================================
// 赛段调试模式（都不定义 = 正式比赛从第1段跑完整6段）
// ============================================================

// #define DEBUG_SINGLE_STAGE 1
// #define DEBUG_START_STAGE  4
// #define DEBUG_END_STAGE    6

// 调试开关：禁用撞球，只观察视觉效果
// #define DEBUG_NO_HIT

// ============================================================
// 行为测试模式（定义后替代状态机，独立测试单个算法/功能）
// ============================================================

// #define DEBUG_TEST_BEHAVIOR
// #define TEST_BEHAVIOR 1
// 1=跳跃 2=扫描找球 3=蹲下 4=路径点导航 5=原地转向 6=起立趴下 7=俯仰角 8=步高

// ============================================================
// Web 推流开关（把摄像头画面通过 HTTP MJPEG 推到网页上）
// 定义 ENABLE_WEB_STREAMING 后编译，race_controller 启动时自动开启 HTTP 服务
// 浏览器访问 http://<IP>:8080 即可查看实时画面
// 正式比赛前注释掉，重新 build = 零额外开销
// ============================================================

#define ENABLE_WEB_STREAMING
#define WEB_STREAM_PORT 8080

// ============================================================
// 仿真 / 真机模式切换
// 定义 REAL_DOG 后编译，适配真实 CyberDog 2 硬件
// 不定义 = 仿真模式（Gazebo），注释即仿真
// ============================================================

// #define REAL_DOG

// ── 传感器 topic 名称（真机值依据官方 developer_guide 文档） ──
// ⚠️ 上机前必读：以下相机 topic 依赖对应的 ROS2 节点已 lifecycle 激活，否则收不到数据！
//   ros2 lifecycle set /stereo_camera configure && ros2 lifecycle set /stereo_camera activate  # RGB相机 /image_rgb
//   ros2 lifecycle set /camera/camera configure && ros2 lifecycle set /camera/camera activate  # Realsense D435
//   ros2 lifecycle set /camera/camera_align configure && ros2 lifecycle set /camera/camera_align activate  # D435对齐彩色
#ifdef REAL_DOG
  #define TOPIC_RGB_CAMERA    "/image_rgb"                     // RGB+鱼眼节点，编码 rgb8（⚠ 需 lifecycle 激活）
  #define TOPIC_IMU           "/imu"                           // 身体IMU（Realsense另有/camera/imu）
  #define TOPIC_LIDAR         "/scan"                          // sensor_msgs/LaserScan（⚠ 真狗可能发 ScanMsg，上机后 ros2 topic info /scan 确认 type）
  #define TOPIC_D435          "/camera/aligned_depth_to_extcolor/image_raw"  // D435对齐彩色（⚠ 需 align 节点 lifecycle 激活）
  #define TOPIC_D435_DEPTH    "/camera/depth/image_rect_raw"   // D435深度图
  #define TOPIC_BMS           "bms_status"                    // 电池（需 protocol::msg::BmsStatus，来自真狗bridges包）
  #define TOPIC_TOUCH         "touch_status"                  // 触摸（⚠ 需 protocol::msg::TouchStatus，非 std_msgs::Int32！拿不到bridges包前禁用）
  // ⚠️ 编码注意事项：on_rgb() 统一要求 cv_bridge 输出 BGR，cv_bridge 自动处理源编码转换（rgb8→bgr8）。
  //    所有检测器(LaneDetector/BallDetector/Stage4Detector)和调试画面均基于 BGR 颜色空间，
  //    只要 on_rgb 拿到的是 BGR 图，下游无需区分真机/仿真。上机后若画面颜色异常，执行:
  //    ros2 topic echo /image_rgb --once | grep encoding  确认源编码是否为 rgb8。
  // LCM 通道（真狗里程计走 LCM global_to_robot，没有 ROS2 /odom）
  #define LCM_ODOM_CHANNEL    "global_to_robot"  // localization_lcmt, 50Hz, 7667
  #define LCM_STATE_ESTIMATOR "state_estimator"
  #define LCM_CMD_EXEC        "exec_request"
#else
  #define TOPIC_RGB_CAMERA    "/RGB_camera/image_raw"
  #define TOPIC_IMU           "/imu"
  #define TOPIC_LIDAR         "/scan"
  #define TOPIC_D435          "/D435/color/image_raw"
#endif

// ============================================================
// LLM 大模型控制（二选一，都不定义=不启用）
// LLM_MODE_PROXY : 狗→ROS2 Srv→笔记本 llm_bridge→云端 API（比赛推荐）
// LLM_MODE_API   : 狗→libcurl→云端 API 直连
// ============================================================

#define LLM_MODE_PROXY
// #define LLM_MODE_API

#if defined(LLM_MODE_PROXY) && defined(LLM_MODE_API)
#error "LLM_MODE_PROXY and LLM_MODE_API are mutually exclusive"
#endif

#ifdef LLM_MODE_PROXY
  #define LLM_SERVICE_NAME  "/llm_ask"
  #define LLM_TIMEOUT       3
#endif
#ifdef LLM_MODE_API
  #define LLM_DEFAULT_URL   "https://api.deepseek.com/v1/chat/completions"
  #define LLM_DEFAULT_MODEL "deepseek-chat"
  #define LLM_TIMEOUT       5
#endif
