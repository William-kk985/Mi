#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第四赛段：深隧寻珍
// 横向通道导航 + 三竖向通道搜索 + 目标交互 + 障碍避让
class Stage4 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    bool is_done() override;

private:
    int cur_channel_{0};  // 当前竖向通道 0/1/2
    bool coke_done_{false};
    bool orange_ball_done_{false};
    bool football_done_{false};
    bool done_{false};

    enum class State {
        TRAVERSE,       // 横向通道行走
        ENTER_CHANNEL,  // 进入竖向通道
        SEARCH,         // 搜索目标
        INTERACT,       // 执行交互
        EXIT_CHANNEL,   // 退出竖向通道
        TO_BRIDGE       // 走向独木桥
    } state_{State::TRAVERSE};
};
