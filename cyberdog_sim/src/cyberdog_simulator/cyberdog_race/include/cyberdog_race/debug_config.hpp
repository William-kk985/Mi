#pragma once

// ============================================================
// 日志/可视化开关
// ============================================================

// #define DEBUG_VISION    // cv::imshow 弹窗看传感器画面
// #define DEBUG_MOTION
// #define DEBUG_SENSOR
// #define DEBUG_STAGE

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
// 9=传感器检查 10=RGB预览 11=原地踏步(servo 303 vel=0)

// ============================================================
// Web 推流 — 全传感器 MJPEG 实时画面
// 浏览器 http://<IP>:8080
// ============================================================

#define ENABLE_WEB_STREAMING
#define WEB_STREAM_PORT 8080

// ============================================================
// 仿真 / 真机模式切换
// ============================================================

// #define REAL_DOG  // 仿真模式
#define REAL_DOG     // 真机模式

// ── 真狗命名空间 ──
#define ROBOT_NS "/mi_desktop_48_b0_2d_7b_02_c7"

// ── 测试模式：11=原地踏步（WALK_USERTROT vel=0） ──
// ⚠ 测试完必须注释回：开着会让 control_loop 跑一次就 timer->cancel()，遥测冻结在启动快照
//   2026-08-07 真机验证 Web 时发现（超声/TOF 卡初始值就是它导致）
#define DEBUG_TEST_BEHAVIOR
#define TEST_BEHAVIOR 15  // 15=绝对转向(地图坐标系, abs_yaw闭环转90°)

// ── 传感器 topic 名称（真机值经 2026-07-31 上机确认） ──
// 相机全部已 lifecycle 激活。RGB 需额外通过 camera_service START_IMAGE_PUBLISH 启动推流
//   模组1 前置AI相机(/stereo_camera): /image(RGB, camera_server推流) 无独立鱼眼topic
//   模组2 底部D430i(/camera/camera):  /camera/infra1(红外) /camera/depth(深度) /camera/imu(IMU)
// ⚠ D430i 无 RGB 彩色输出，只有红外+深度！
#ifdef REAL_DOG
  #define TOPIC_RGB_CAMERA       ROBOT_NS "/image"                       // AI相机 RGB（camera_server推流, 编码rgb8）
  #define TOPIC_IMU              ROBOT_NS "/camera/imu"                  // D430i 内置IMU（真狗无独立身体IMU topic）
  #define TOPIC_LIDAR            ROBOT_NS "/scan"                        // sensor_msgs/LaserScan ✅
  #define TOPIC_D435_INFRA1      ROBOT_NS "/camera/infra1/image_rect_raw" // D430i左目红外 灰度 ✅
  #define TOPIC_D435_DEPTH       ROBOT_NS "/camera/depth/image_rect_raw"  // D430i深度图 mono16(mm) ✅
  #define TOPIC_BMS              ROBOT_NS "/bms_status"                  // protocol::msg::BmsStatus ✅
  #define TOPIC_TOUCH            ROBOT_NS "/touch_status"                // protocol::msg::TouchStatus ✅
  #define TOPIC_D435_INFRA2      ROBOT_NS "/camera/infra2/image_rect_raw" // D430i右目红外 mono8 ✅(2026-08-06探测)
  #define TOPIC_TOF_HEAD         ROBOT_NS "/head_tof_payload"            // 头TOF×2 8x8高程 ✅
  #define TOPIC_TOF_REAR         ROBOT_NS "/rear_tof_payload"            // 尾TOF×2 8x8高程 ✅
  #define TOPIC_ULTRASONIC       ROBOT_NS "/ultrasonic_payload"          // 超声 Range ✅
  // LCM 通道（真狗可走LCM global_to_robot，也可用ROS2 odom_out替代）
  #define LCM_ODOM_CHANNEL    "global_to_robot"
  #define LCM_STATE_ESTIMATOR "state_estimator"
  #define LCM_CMD_EXEC        "exec_request"
#else
  #define TOPIC_RGB_CAMERA       "/RGB_camera/image_raw"
  #define TOPIC_IMU              "/imu"
  #define TOPIC_LIDAR            "/scan"
  #define TOPIC_D435_INFRA1      "/D435/infra1/image_raw"  // 仿真Gazebo D435红外
  #define TOPIC_D435_DEPTH       "/D435/depth/image_raw"
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
