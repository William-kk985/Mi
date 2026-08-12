#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第3赛段: 低头 + 视觉检测 (测试形态, 不前进)
/// 低头: 201姿态控制 set_body_pitch (原地无步态夹持, 可到~14°, 无需破限)
/// 视觉: LaneDetector 正常跑(on_rgb /image_rgb), Web 标注看检测效果
/// ⚠ 测试形态: 原地低头不动, 便于调视觉; 正式巡线待验证 (2026-08-12)
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
    int   pitch_hold_{0};      // 201低头发布计数 (控制循环100Hz, 每3帧≈33Hz保持)
};
