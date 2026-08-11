#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第2赛段
/// 前进 1.5m → 左转 90° → 再前进 3.6m
class Stage2Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { FWD1, TURN, FWD2, DONE };

    Phase phase_{Phase::FWD1};
    bool  done_{false};
    float start_yaw_{0.0f};    // 赛段初始朝向 (左转90°基准)
    float fwd1_x_{0.0f}, fwd1_y_{0.0f};   // FWD1 起点
    float fwd2_x_{0.0f}, fwd2_y_{0.0f};   // FWD2 起点
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};   // 累计前进位移
    int   turn_guard_{0};    // 转弯最小帧数保护
};
