#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第六赛段实际实现（雷达定位 + 推球）
// TODO: 后续替换虚拟赛段
class Stage6Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override {}
    void run() override {}
    [[nodiscard]] bool is_done() override { return false; }
};
