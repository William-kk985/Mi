#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第四赛段实际实现（YOLO + 视觉）
// TODO: 后续替换虚拟赛段
class Stage4Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override {}
    void run() override {}
    [[nodiscard]] bool is_done() override { return false; }
};
