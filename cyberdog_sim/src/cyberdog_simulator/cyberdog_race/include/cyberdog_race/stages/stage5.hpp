#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第五赛段：孤梁稳渡
// 黄线巡线居中 + D435深度兜底 + 虚线检测 + 跳下
class Stage5 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    bool is_done() override;

private:
    bool done_{false};
    bool crossed_line_{false};  // 四足是否越过虚线

    enum class State {
        ON_BRIDGE,   // 桥上行走
        WAIT_CROSS,  // 等待四足越线
        JUMP         // 执行跳下
    } state_{State::ON_BRIDGE};
};
