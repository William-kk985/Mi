#pragma once
#include <lcm/lcm-cpp.hpp>
#include "cyberdog_race/gamepad_lcmt.hpp"
#include "robot_control_cmd_lcmt.hpp"

class MotionCtrl {
public:
    MotionCtrl();

    // 速度控制：x前后，y左右，yaw转向（正=左转，负=右转）
    void set_velocity(float x, float y, float yaw);
    // 俯仰角：负值低头，正值抬头
    void set_pitch(float pitch);
    // 分别设置左右侧步高（单位：m），step_height[0]=前/左侧，step_height[1]=后/右侧
    void set_step_height(float left, float right);

    // 模式切换
    void stand();        // QP站立
    void locomotion();   // 行走模式
    void lie_down();     // 趴下
    void recovery();     // 恢复站立
    void stop();         // 停止（发零速）
    void jump();                      // kJump3d (mode=16) gait_id=4, 离线轨迹跳跃
    void force_jump();               // kForceJump (mode=22) gait_id=4, 力控跳跃
    void send_lcm_mode(int mode, int gait_id = 0); // 通过robot_control_cmd LCM通道发送模式命令

private:
    lcm::LCM              lcm_;           // 默认7667端口，给gamepad用
    lcm::LCM              ctrl_lcm_;      // 7671端口，给robot_control_cmd用
    gamepad_lcmt          gpad_;
    robot_control_cmd_lcmt lcm_cmd_;
    int                   lcm_life_{0};
    void pub_gamepad();
    void pub_lcm_cmd();
};
