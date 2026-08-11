#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第2赛段
/// 先前进 3.6m → 再左转 90°(与 Stage1 对称: 直行→转)
class Stage2Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { FORWARD, TURN, DONE };

    Phase phase_{Phase::FORWARD};
    bool  done_{false};
    float start_yaw_{0.0f};
    float start_x_{0.0f}, start_y_{0.0f};
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};   // 累计前进位移
    int   turn_guard_{0};    // 转弯最小帧数保护
};
