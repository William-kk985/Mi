#pragma once
#include "cyberdog_race/motion_ctrl.hpp"

// 传感器数据共享结构，由主线程填充
struct SensorData {
    // 视觉结果（视觉线程写，主线程读）
    float lane_offset{0.0f};    // 黄线中心偏差，正=偏右
    float lane_yaw{0.0f};       // 黄线方向偏差
    float lane_curvature{0.0f}; // 弯曲程度（斜率标准差）
    bool  lane_valid{false};    // 黄线是否有效
    bool  lane_both_sides{false}; // 是否双边检测

    float ball_x{0.0f};        // 球在图像中的x坐标（归一化）
    float ball_y{0.0f};
    float ball_dist{0.0f};     // 球距离（米）
    bool  ball_found{false};   // 是否找到目标球

    // IMU
    float yaw{0.0f};           // 当前偏航角（rad）
    float pitch{0.0f};
    float roll{0.0f};

    // 里程计
    float odom_x{0.0f};
    float odom_y{0.0f};

    // Lidar：前方最近障碍距离
    float lidar_front{10.0f};
    float lidar_left{10.0f};
    float lidar_right{10.0f};
};

class StageBase {
public:
    explicit StageBase(MotionCtrl& motion, SensorData& sensor)
        : motion_(motion), sensor_(sensor) {}
    virtual ~StageBase() = default;

    virtual void init() = 0;
    virtual void run() = 0;
    virtual bool is_done() = 0;

protected:
    MotionCtrl&  motion_;
    SensorData&  sensor_;
};
