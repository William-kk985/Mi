#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第六赛段：撷金建功
// Lidar找出口 + 相机找足球 + 推球出口 + 走终点趴下
class Stage6 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    bool is_done() override;

private:
    bool done_{false};
    float exit_angle_{0.0f};  // Lidar检测到的出口方向

    enum class State {
        FIND_EXIT,    // Lidar扫描找出口
        FIND_BALL,    // 相机找足球
        PUSH_BALL,    // 推球到出口
        KICK,         // 踢出
        TO_FINISH,    // 走向终点
        LIE_DOWN      // 趴下结束
    } state_{State::FIND_EXIT};
};
