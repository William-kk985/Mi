#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第三赛段：曲道冲锋
// 复用黄线巡线，S形曲道，弯道降速
class Stage3 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    bool is_done() override;

private:
    bool done_{false};
};
