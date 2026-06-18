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
