#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第3赛段: 破限低头前进
/// 移植 test19 pitch_unlock_test (真机 ✅ 14°低头保持走满0.5m, 2026-08-08)
/// LCM 设 x_effect_scale_pos=+30 放大走路pitch限位 → 303前进+低头0.25 保持走满
/// ⚠ 破限参数同时放大 y/yaw 限位, 只直行, 完成必须复原 -0.55
/// 真机约定: 正值=低头 (2026-08-08 舵机方向确认)
class Stage3Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { WAIT_READY, LOW_WALK, DONE };

    Phase phase_{Phase::WAIT_READY};
    bool  done_{false};
    bool  loc_ready_{false};   // global_to_robot 定位就绪 (spin后才有数据, 同Stage1)
    bool  unlock_set_{false};  // 破限参数是否已设置 (确保复原)
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};     // 累计位移 (防定位跳变)
};
