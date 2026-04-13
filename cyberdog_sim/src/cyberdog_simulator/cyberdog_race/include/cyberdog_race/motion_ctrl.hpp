#pragma once
#include <lcm/lcm-cpp.hpp>
#include "cyberdog_race/gamepad_lcmt.hpp"

class MotionCtrl {
public:
    MotionCtrl();

    // 设置速度：x前后，y左右，yaw转向，范围[-1,1]
    void set_velocity(float x, float y, float yaw);
    void set_pitch(float pitch);  // 俯仰角，负值低头，范围约[-0.9, 0.9]

    // 模式切换
    void stand();        // QP stand
    void locomotion();   // 行走模式
    void lie_down();     // 趴下
    void recovery();     // 恢复站立

    // 停止
    void stop();

private:
    lcm::LCM lcm_;
    gamepad_lcmt cmd_;
    void publish();
};
