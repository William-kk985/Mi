#pragma once

#include <vector>

// 传感器数据共享结构，由主循环填充并由各赛段只读访问
struct SensorData {
    // 视觉结果（视觉线程写，主线程读）
    float lane_offset{0.0f};    // 黄线中心偏差，正=偏右
    float lane_curvature{0.0f}; // 弯曲程度（斜率标准差）
    bool  lane_valid{false};    // 黄线是否有效
    bool  lane_both_sides{false}; // 是否双边检测

    float ball_x{0.0f};        // 球在图像中的x坐标（归一化）
    float ball_dist{0.0f};     // 球距离（米）
    bool  ball_found{false};   // 是否找到目标球

    float blue_ball_x{0.0f};   // 蓝球x坐标（归一化）
    float blue_ball_dist{0.0f};
    bool  blue_ball_found{false};

    float white_ball_x{0.0f};    // 白球x坐标（归一化）
    float white_ball_dist{0.0f};
    bool  white_ball_found{false};

    // IMU
    float yaw{0.0f};           // 当前偏航角（rad）
    float pitch{0.0f};
    float roll{0.0f};

    // 里程计
    float odom_x{0.0f};
    float odom_y{0.0f};
    float body_height{0.25f};  // 身体离地高度（m）
    float abs_yaw{0.0f};       // 地图坐标系绝对朝向（global_to_robot.rpy[2], SLAM固定坐标系）
    float pitch_map{0.0f};     // 地图坐标系俯仰（global_to_robot.rpy[1], 2026-08-08 抬头验证用）
    float roll_map{0.0f};      // 地图坐标系横滚/侧倾（global_to_robot.rpy[0], 2026-08-08 身躯倾斜验证用）

    // Lidar：前方最近障碍距离
    float lidar_front{10.0f};

    // TOF 四腿离地间隙（head/rear 回调写入, 2026-08-06 已接入 protocol 消息）
    // 四个TOF最低点 (m), 有效范围 0.15-0.66, 用于Stage5独木桥检测
    float tof_clearance{0.66f};

    // 超声测距（ultrasonic_payload, 2026-08-06 接入）
    float ultrasonic_range{0.0f};  // 0 = 无效/未收到
};
