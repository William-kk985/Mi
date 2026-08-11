#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第2赛段
/// 走到相对目标点: 起点右转 5°(修正 Stage1 转向偏差)方向 1m 处
/// 用 StageBase 通用 goto_relative (不借助地图)
class Stage2Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { NAV, DONE };

    Phase phase_{Phase::NAV};
    bool  done_{false};
};
