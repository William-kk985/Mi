#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第3赛段: 破限低头 + 视觉检测 (测试形态, 不前进)
/// 破限低头: LCM x_effect_scale_pos=+30 → 低头0.25 原地保持 (test19 验证)
/// 视觉: LaneDetector 正常跑(on_rgb /image), Web 标注看检测效果
/// ⚠ 测试形态: 原地低头不前进, 便于调视觉; 正式巡线待验证 (2026-08-12)
class Stage3Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { WAIT_READY, LOW_HOLD };

    Phase phase_{Phase::WAIT_READY};
    bool  done_{false};
    bool  loc_ready_{false};   // 定位就绪 (spin后才有数据, 同Stage1)
    bool  unlock_set_{false};  // 破限参数是否已设置 (确保复原)
};
