#pragma once

// ============================================================
// 日志/可视化开关
// ============================================================

// #define DEBUG_VISION    // cv::imshow 弹窗看传感器画面 (⚠真机无显示器, 勿开)
#define DEBUG_MOTION     // 运动指令日志 (2026-08-11 真机调试开)
#define DEBUG_SENSOR     // 传感器数据日志 (odom/yaw/dist)
#define DEBUG_STAGE      // 状态机切换日志

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

// #define DEBUG_SINGLE_STAGE 3   // 只跑 Stage3, 结束后停止不切换 (2026-08-15 巡线拍照调试)
// #define DEBUG_START_STAGE  3   // 从Stage3开始 (2026-08-18 之前: 只走Stage3+4)
#define DEBUG_END_STAGE    5   // 跑完Stage5停止 (2026-08-20 用户: 走Stage1~5兼容伙伴Stage5)

// (2026-08-21 startrace4test专用: Stage4左横向补偿全0, 观察自然偏左; 正式版保持注释)
// #define DEBUG_STAGE4_NO_COMP

// (2026-08-22 startrace4test/123456test: Stage4走test路线(起步1.8/1.0/1.8, 第3轮绕行, 离场左转→0.5m→左转); 正式版保持注释)
// #define DEBUG_STAGE4_TEST_ROUTE

// (2026-08-22 startrace4test2: Stage4走test路线2/第三版(轮1:0.8→3.5/3.4→前1.8; 轮2:1.1→3.35/3.3→左转→前2.5→左转立正); 与TEST_ROUTE互斥)
// #define DEBUG_STAGE4_TEST_ROUTE2

// (2026-08-21 startrace2test专用: 离场3.3m前左转2°(原4°减半); 正式版保持注释)
// #define DEBUG_STAGE2_TEST

// 调试开关：禁用撞球，只观察视觉效果
// #define DEBUG_NO_HIT

// ============================================================
// 行为测试模式（定义后替代状态机，独立测试单个算法/功能）
// ============================================================

// #define DEBUG_TEST_BEHAVIOR
// #define TEST_BEHAVIOR 1
// 1=跳跃 2=扫描找球 3=蹲下 4=路径点导航 5=原地转向 6=起立趴下 7=俯仰角
// 8=步高(旧接口,已弃用) 9=传感器检查 10=RGB预览 11=原地踏步
// 12=前进0.5m 13=前跳30cm 14=相对转向90° 15=绝对转向90° 16=步高标定(打包毫米,0.15基准)
// 17=低头前进(303带rpy_des[1]=pitch+前进,步态读rpy_des保持低头) 18=roll走路侧倾(YamlParam des_roll_pitch_height[0])
// 19=pitch破限(x_effect_scale_pos=+30 放大走路pitch限位) 20=降低身高前进(TOF确认收敛再走)
// 完整清单/要点/坑 见 README！.md「行为测试」

// ============================================================
// Web 推流 — 全传感器 MJPEG 实时画面
// 浏览器 http://<IP>:8080
// ============================================================

#define ENABLE_WEB_STREAMING
#define WEB_STREAM_PORT 8080

// ── 关闭 D430i 红外/深度 (任务只用RGB, 关闭减少CPU/资源竞争, 2026-08-13) ──
#define DISABLE_D435_SUB

// ── RGB 相机通道 (2026-08-14) ──
// center 模组 (ov9782 双目, cam_id 2/3) 视角更适合寻线; bottom 相机(camera_server)位置不好
#define USE_CENTER_CAM

// ============================================================
// 仿真 / 真机模式切换
// ============================================================

// #define REAL_DOG  // 仿真模式
#define REAL_DOG     // 真机模式

// ── 真狗命名空间 ──
#define ROBOT_NS "/mi_desktop_48_b0_2d_7b_02_c7"

// ── 当前测试：20=降低身高前进（pos_des[2] 扫描诊断，TOF活反馈确认） ──
// ⚠ 测试完必须注释回 DEBUG_TEST_BEHAVIOR：开着会让 control_loop 只跑测试一次后
//   timer->cancel()，遥测冻结在启动快照（超声卡0/TOF卡0.66）
//   （2026-08-08 已改独立线程跑测试，期间传感器实时，但跑完仍会 cancel）
// #define DEBUG_TEST_BEHAVIOR
// #define TEST_BEHAVIOR 21

// ── 传感器 topic 名称（真机值经 2026-07-31 上机确认） ──
// 相机全部已 lifecycle 激活。RGB 需额外通过 camera_service START_IMAGE_PUBLISH 启动推流
//   模组1 前置AI相机(/stereo_camera): /image(RGB, camera_server推流) 无独立鱼眼topic
//   模组2 底部D430i(/camera/camera):  /camera/infra1(红外) /camera/depth(深度) /camera/imu(IMU)
// ⚠ D430i 无 RGB 彩色输出，只有红外+深度！
#ifdef REAL_DOG
  #ifdef USE_CENTER_CAM
    #define TOPIC_RGB_CAMERA       ROBOT_NS "/image_center"        // gc02m1彩色鼻区相机 (2026-08-14定稿: cam_id=1 30fps)
  #else
    #define TOPIC_RGB_CAMERA       ROBOT_NS "/image"               // bottom camera_server推流 (备用)
  #endif
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
  #define TOPIC_ODOM_OUT         ROBOT_NS "/odom_out"                    // RT板腿里程计+融合IMU姿态 46.6Hz (2026-08-15 航向反馈源)
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
