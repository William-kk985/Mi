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
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};   // 累计前进位移 (防定位跳变误判6m, 2026-08-11)
    int   turn_guard_{0};    // 转弯最小帧数保护 (防 yaw 跳变立即到位)
    bool  loc_ready_{false}; // global_to_robot 定位是否就绪 (spin后才有数据, 2026-08-12)
};
