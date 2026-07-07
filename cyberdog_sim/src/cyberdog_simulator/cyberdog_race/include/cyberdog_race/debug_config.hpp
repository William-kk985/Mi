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

// #define ENABLE_WEB_STREAMING
#define WEB_STREAM_PORT 8080

// ============================================================
// 仿真 / 真机模式切换
// 定义 REAL_DOG 后编译，适配真实 CyberDog 2 硬件
// 不定义 = 仿真模式（Gazebo），注释即仿真
// ============================================================

// #define REAL_DOG

// ── 传感器 topic 名称（REAL_DOG 下的值需 SSH 进真狗 ros2 topic list 确认后修改） ──
#ifdef REAL_DOG
  #define TOPIC_RGB_CAMERA    "/camera/color/image_raw"        // TODO: 确认真狗 RGB 相机 topic
  #define TOPIC_IMU           "/imu"                           // TODO: 确认，大概率不变
  #define TOPIC_LIDAR         "/scan"                          // TODO: 确认真狗 LiDAR topic（可能是 /laser_scan）
  #define TOPIC_D435          "/camera/depth/color/image_raw"  // TODO: 确认真狗 D435 相机 topic
  #define TOPIC_ODOM          "/odom"                          // TODO: 确认真狗里程计 topic
  // LCM 通道（真狗用 state_estimator 替代 simulator_state）
  #define LCM_STATE_ESTIMATOR "state_estimator"
  #define LCM_CMD_EXEC        "exec_request"
#else
  #define TOPIC_RGB_CAMERA    "/RGB_camera/image_raw"
  #define TOPIC_IMU           "/imu"
  #define TOPIC_LIDAR         "/scan"
  #define TOPIC_D435          "/D435/color/image_raw"
#endif
