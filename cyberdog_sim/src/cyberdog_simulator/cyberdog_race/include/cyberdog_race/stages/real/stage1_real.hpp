#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版
/// 独立实现，与仿真版本互不干扰
class Stage1Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override {}
    void run() override {}
    [[nodiscard]] bool is_done() override { return false; }
};
