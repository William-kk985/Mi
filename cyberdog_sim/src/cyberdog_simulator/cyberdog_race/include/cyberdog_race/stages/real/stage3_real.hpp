#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第3赛段: 低头 + 视觉巡线 (2026-08-13 正式形态)
/// 低头: 破限(x_effect_scale_pos=+30) + 303 rpy_des[1]=pitch 前进低头 (test17已验证)
///   ⚠ 破限只在 vx>0 时生效 → 巡线持续前进
/// 巡线: lane_offset(>0=车偏左) → yaw_cmd=-KP*offset 回中; 丢线→直行
/// 测试形态(TEST_HOLD=true): 201原地低头不动, 调视觉用
class Stage3Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { WAIT_READY, LANE_FOLLOW, DONE };

    Phase phase_{Phase::WAIT_READY};
    bool  done_{false};
    bool  loc_ready_{false};   // 定位就绪 (spin后才有数据, 同Stage1)
    int   pitch_hold_{0};      // 201低头发布计数 (测试形态用)
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};     // 巡线累计位移
    float last_offset_{0.0f};  // 上帧 lane_offset (2026-08-13 微分预测项)
    float last_yaw_{0.0f};     // 丢线保持的最后转向 (2026-08-13 赛道出画面时继续转拉回)
};
