#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第3赛段: 写死路径 (2026-08-15 用户指定)
/// 路径(相对转向+前进): 右转30°前0.3m → 右转50°前0.2m → 右转10°前2m
///                      → 左转50°前0.3m → 左转30°前0.8m
/// 转向: abs_yaw 闭环(右转=负); 前进: odom 距离 + 航向锁
class Stage3Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    // SETTLE: (2026-08-17) 走完收尾——303静止抬平0.5s再DONE, 防Stage3→4切换摔
    enum class Phase { WAIT_READY, TURN, FWD, SETTLE, DONE };

    Phase phase_{Phase::WAIT_READY};
    bool  done_{false};
    int   step_idx_{0};       // 当前路径步 0~4
    float target_yaw_{0.0f};  // 本步转向目标(绝对)
    int   turn_settle_{0};    // 转到位停稳帧数
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};    // 本步前进累计

    // ── 矫正项 (2026-08-16 与 Stage2 对齐) ──
    int   turn_guard_{0};                 // 停稳复核补转计数(最多60帧)
    float drift_rate_{0.0f};              // yaw 漂移率前馈低通(rad/s)
    float last_yaw_err_{0.0f};            // 上帧航向误差(算漂移率)
    float step_start_x_{0.0f}, step_start_y_{0.0f};  // 本前进步起点(odom)
    float step_cos_{1.0f}, step_sin_{0.0f};          // 本前进步朝向(航向锁目标)
};
