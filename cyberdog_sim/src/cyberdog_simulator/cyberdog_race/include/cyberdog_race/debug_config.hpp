#pragma once

// ============================================================
// 日志/可视化开关
// 正式比赛前全部注释掉，重新 build = 零额外开销
// ============================================================

#define DEBUG_VISION    // 视觉可视化（cv::imshow，黄线边界，中心线，球检测）
#define DEBUG_MOTION    // 运动控制日志（速度指令输出）
#define DEBUG_SENSOR    // 传感器数据日志（IMU/里程计/Lidar）
#define DEBUG_STAGE     // 绿色终端打印（受 DEBUG_STAGE 控制）
#ifdef DEBUG_STAGE
#define LOG_STAGE_GREEN(tag, msg) fprintf(stderr, "\033[1;34m[" tag "] " msg "\033[0m\n")
#define LOG_STAGE_GREENF(tag, fmt, ...) fprintf(stderr, "\033[1;34m[" tag "] " fmt "\033[0m\n", ##__VA_ARGS__)
#else
#define LOG_STAGE_GREEN(tag, msg)
#define LOG_STAGE_GREENF(tag, fmt, ...)
#endif

// ============================================================
// 赛段调试模式（都不定义 = 正式比赛从第1段跑完整6段）
//
// DEBUG_SINGLE_STAGE N : 只跑第N赛段，完成后自动停止，不切换下一段
// DEBUG_START_STAGE  N : 从第N赛段开始，正常跑完剩余所有赛段
// DEBUG_END_STAGE    N : 从第1赛段开始，跑完第N赛段后停止
//
// 改完之后必须重新 colcon build 才生效
// ============================================================

// #define DEBUG_SINGLE_STAGE 1
// #define DEBUG_START_STAGE  1
#define DEBUG_END_STAGE    2
