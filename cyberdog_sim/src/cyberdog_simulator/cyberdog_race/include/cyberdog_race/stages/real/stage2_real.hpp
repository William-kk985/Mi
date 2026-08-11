#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第2赛段
/// 先右转 5°(修正 Stage1 转向偏差) → 前进 1.5m
/// 独立实现，与仿真版本互不干扰
class Stage2Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { CORRECT, FORWARD, DONE };

    Phase phase_{Phase::CORRECT};
    bool  done_{false};
    int   correct_guard_{0};   // 右转修正最小帧数(防 yaw 跳变立即到位)
    float start_yaw_{0.0f};
    float start_x_{0.0f}, start_y_{0.0f};
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};     // 累计前进位移
};
