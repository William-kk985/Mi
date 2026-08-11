#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第2赛段
/// 两个相对点位 (goto_relative):
///   点位1: 右转 5° 走 1.2m
///   点位2: 左转 85° 走 3.4m
class Stage2Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { NAV1, NAV2, DONE };

    Phase phase_{Phase::NAV1};
    bool  done_{false};
};
