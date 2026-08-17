#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第1赛段 石径探路
/// 步高 0.15 | 前进 6m | 视觉巡线 + 里程计 + IMU 转90°
/// 独立实现，与仿真版本互不干扰
class Stage1Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { FORWARD, TURN, DONE };

    Phase phase_{Phase::FORWARD};
    bool  done_{false};
    int   stuck_{0};       // 卡住帧计数
    int   rush_{0};        // 冲刺剩余帧
    int   lane_lost_{0};   // 丢线帧计数
    float prev_offset_{0.0f};
    float start_x_{0.0f}, start_y_{0.0f}, start_yaw_{0.0f};
    float start_odom_yaw_{0.0f};   // 起点odom系航向 (2026-08-16 前进回正换高频odom源, 防卡脚偏航反馈慢)
    float start_imu_yaw_{0.0f};    // 起点external_imu航向 (2026-08-16 石径回正抗抖源)
    float lat_guard_prev_{0.0f};  // SLAM漂移检测: 上一帧|latDev| (2026-08-16)
    int   lat_guard_cnt_{0};      // 满幅且恶化连续帧计数
    bool  lat_hold_{false};       // SLAM漂移保护: 横向纠正冻结
    int   loc_stable_cnt_{0};     // 定位稳定帧计数 (锁起点前收敛判据)
    float loc_prev_x_{0.0f}, loc_prev_y_{0.0f};
    float target_x_{0.0f}, target_y_{0.0f};  // 前进目标点 (2026-08-16 点位闭环)
    int   step_time_{0};           // 前进已走帧数 (2026-08-16 到位判定指令积分下限)
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};   // 累计前进位移 (防定位跳变误判6m, 2026-08-11)
    int   turn_guard_{0};    // 转弯最小帧数保护 (防 yaw 跳变立即到位)
    int   turn_settle_{0};   // 转向停稳确认帧 (2026-08-12 提高转向精度)
    bool  loc_ready_{false}; // global_to_robot 定位是否就绪 (spin后才有数据, 2026-08-12)
};
