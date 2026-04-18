#pragma once
#include <lcm/lcm-cpp.hpp>
#include "cyberdog_race/gamepad_lcmt.hpp"

class MotionCtrl {
public:
    MotionCtrl();

    // 速度控制：x前后，y左右，yaw转向（正=左转，负=右转）
    void set_velocity(float x, float y, float yaw);
    // 俯仰角：负值低头，正值抬头
    void set_pitch(float pitch);

    // 模式切换
    void stand();        // QP站立
    void locomotion();   // 行走模式
    void lie_down();     // 趴下
    void recovery();     // 恢复站立
    void stop();         // 停止（发零速）

private:
    lcm::LCM     lcm_;
    gamepad_lcmt gpad_;
    void pub_gamepad();
};
