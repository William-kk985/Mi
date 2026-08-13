#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第2赛段 (2026-08-14 侧移扫球重构)
/// 流程: 前进0.92m → 轮1左/轮2右/轮3左/轮4右 各侧移2.8m(边走边找球)
///       轮间前进0.75m衔接; 轮4走满直接结束(不前进0.75)
/// 全程不转向, 侧移走 303 vel_des.y (机器人系, y正=左假设)
/// 找球: 侧移中球距≤0.7m连续确认12帧 → 中断朝球冲击(距离闭环) → 退回0.2m
///       → 继续走完本轮侧移剩余(不再找球, 防漏扫+防重撞) → 前进0.75进下一轮
class Stage2Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { FWD0, SLIDE, IMPACT, BACK, FWD_GAP, DONE };

    Phase phase_{Phase::FWD0};
    bool  done_{false};
    int   round_{0};            // 轮次 0/1/2/3 (轮1~轮4)
    int   ball_confirm_{0};     // 球连续确认帧数 (防误检)
    bool  hit_this_round_{false};  // 本轮已撞击过 → 剩余侧移只走完不再找球 (2026-08-14 防重撞)
    float slide_left_{0.0f};    // 本轮侧移剩余距离
    float slide_yaw_ref_{0.0f}; // 侧移航向基准 (2026-08-14 侧移漂移补偿: 锁航向)
    float impact_x_{0.0f}, impact_y_{0.0f};   // 撞击起点 (2026-08-14 回位漂移诊断)
    float impact_yaw_{0.0f};
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};      // 当前子段累计位移
};

