#include "cyberdog_race/motion_ctrl.hpp"
#include <cstring>

MotionCtrl::MotionCtrl() : ctrl_lcm_("udpm://239.255.76.67:7671?ttl=255") {
    memset(&gpad_, 0, sizeof(gpad_));
}

// NOTE: publish-then-reset 模式依赖 LCM 的同步发布语义（lcm_.publish 返回时数据已序列化完毕）。
// 若未来 LCM 切换为异步模式或更换传输层，需在 publish 后加内存屏障或改用双缓冲。
void MotionCtrl::set_velocity(float x, float y, float yaw) {
    gpad_.y                   = 1;
    gpad_.leftStickAnalog[1]  = x;
    gpad_.leftStickAnalog[0]  = -y;
    gpad_.rightStickAnalog[0] = -yaw;
    pub_gamepad();
    gpad_.y                   = 0;
    gpad_.leftStickAnalog[1]  = 0;
    gpad_.leftStickAnalog[0]  = 0;
    gpad_.rightStickAnalog[0] = 0;
}

void MotionCtrl::set_pitch(float pitch) {
    gpad_.rightStickAnalog[1] = pitch;
    pub_gamepad();
    gpad_.rightStickAnalog[1] = 0;
}

void MotionCtrl::stand()      { gpad_.x = 1; pub_gamepad(); gpad_.x = 0; }
void MotionCtrl::locomotion() { gpad_.y = 1; pub_gamepad(); gpad_.y = 0; }
void MotionCtrl::lie_down()   { gpad_.a = 1; pub_gamepad(); gpad_.a = 0; }
void MotionCtrl::recovery()   { gpad_.b = 1; pub_gamepad(); gpad_.b = 0; }

void MotionCtrl::stop() {
    gpad_.y                   = 1;
    gpad_.leftStickAnalog[1]  = 0;
    gpad_.leftStickAnalog[0]  = 0;
    gpad_.rightStickAnalog[0] = 0;
    pub_gamepad();
    gpad_.y = 0;
}

void MotionCtrl::pub_gamepad() {
    lcm_.publish("gamepad_lcmt", &gpad_);
}

void MotionCtrl::pub_lcm_cmd() {
    lcm_cmd_.life_count = ++lcm_life_;
    ctrl_lcm_.publish("robot_control_cmd", &lcm_cmd_);
}

void MotionCtrl::jump() {
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode       = static_cast<int8_t>(LocoMode::JUMP_3D);
    lcm_cmd_.gait_id    = 1;           // JumpId::kJumpPosX60（向前跳60cm）
    lcm_cmd_.vel_des[0] = 0.4f;        // 带前向速度起跳，增加跳跃距离
    pub_lcm_cmd();
}

void MotionCtrl::force_jump() {
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode    = static_cast<int8_t>(LocoMode::FORCE_JUMP);
    lcm_cmd_.gait_id = 4;
    pub_lcm_cmd();
}

void MotionCtrl::send_lcm_mode(int mode, int gait_id) {
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode    = mode;
    lcm_cmd_.gait_id = gait_id;
    pub_lcm_cmd();
}

void MotionCtrl::set_step_height(float left, float right) {
    float left_clamped  = left  < 0.0f ? 0.0f : (left  > 0.35f ? 0.35f : left);
    float right_clamped = right < 0.0f ? 0.0f : (right > 0.35f ? 0.35f : right);
    
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.step_height[0] = left_clamped;
    lcm_cmd_.step_height[1] = right_clamped;
    ctrl_lcm_.publish("robot_control_cmd", &lcm_cmd_);  // P2 fix: 与其他 robot_control_cmd 统一用 7671 端口
}
