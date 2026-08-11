#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第2赛段
/// 右转3°→走1m→左转90°→走2.9m→右转45°→左转45°(回正,不找球)
/// 最后回正让狗朝好方向停下, 等下一赛段继续走
class Stage2Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { TURN1, FWD1, TURN2, FWD2, TURN3, TURN4, DONE };

    Phase phase_{Phase::TURN1};
    bool  done_{false};
    int   turn_guard_{0};      // 转向最小帧数保护
    float turn_base_yaw_{0.0f};   // 进入转向时的朝向 (相对当前转)
    float fwd_ref_yaw_{0.0f};     // 前进段目标朝向 (回正基准)
    float start_yaw_{0.0f};       // 赛段初始朝向
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};        // 累计位移
};
