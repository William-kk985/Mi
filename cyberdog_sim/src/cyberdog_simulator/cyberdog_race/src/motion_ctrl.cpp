// ⚠ debug_config.hpp 必须先于 motion_ctrl.hpp 包含：
//   motion_ctrl.hpp 内的 #ifdef REAL_DOG 块（protocol 发布器声明/成员）依赖 REAL_DOG 宏，
//   若后包含则 REAL_DOG 未定义，声明/实现被跳过 → 链接错误 undefined reference
#include "cyberdog_race/debug_config.hpp"
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

// ── 真机姿态控制 ──
// ⚠️ CyberDog2 正确接口：ROS2 motion_servo_cmd + FORCECONTROL_DEFINITIVELY(201)
//    rpy_des=[roll,pitch,yaw]，pitch 负值=低头(限-0.25)、正值=抬头(限+0.30)
//    pos_des=[0,0,0.235] 机身高度；cmd_source=-1 最高调试优先级
//    需要 ~20Hz 持续发布（停发 4 帧 motion_manager 会判定 Servo data lost 并退出）
//    ❌ 旧的 LCM mode=21 (POSE_CTRL) 是铁蛋一代接口，真机被错误映射成 WALK_USERTROT(303)
void MotionCtrl::set_body_pitch(float pitch) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_body_pitch: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 201;   // MotionID::FORCECONTROL_DEFINITIVELY
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {0.0f, 0.0f, 0.0f};
    cmd.rpy_des     = {0.0f, pitch, 0.0f};
    cmd.pos_des     = {0.0f, 0.0f, 0.235f};
    cmd.step_height = {0.05f, 0.05f};
    motion_servo_pub_->publish(cmd);
#else
    // 仿真：原始 LCM mode=21 POSE_CTRL
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode      = static_cast<int8_t>(LocoMode::POSE_CTRL);  // 21
    lcm_cmd_.gait_id   = 0;
    lcm_cmd_.rpy_des[1] = pitch;   // pitch
    lcm_cmd_.duration  = 500;      // 500ms 内完成姿态过渡
    pub_lcm_cmd();                 // life_count++ + 发布 robot_control_cmd
#endif
}

#ifdef REAL_DOG
void MotionCtrl::attach_motion_servo_pub(rclcpp::Node* node) {
    motion_servo_pub_ = node->create_publisher<protocol::msg::MotionServoCmd>(
        ROBOT_NS "/motion_servo_cmd", rclcpp::QoS(10));
    RCLCPP_INFO(node->get_logger(), "[MotionCtrl] motion_servo_cmd 发布器已挂载(%s)",
                ROBOT_NS "/motion_servo_cmd");
}

// ── 真机行走/原地踏步 ──
// ⚠️ 与 set_body_pitch 同理，走 ROS2 motion_servo_cmd 官方接口（旧 gamepad 是铁蛋一代）
//    WALK_USERTROT(303) + vel_des=[x,y,yaw]；全 0 = 原地踏步
//    需 ~20Hz 持续发布（停发 4 帧 = Servo data lost 退出）；cmd_source=-1 最高优先级
void MotionCtrl::set_walk_velocity(float x, float y, float yaw) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_walk_velocity: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 303;   // MotionID::WALK_USERTROT
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {x, y, yaw};
    cmd.rpy_des     = {0.0f, 0.0f, 0.0f};
    cmd.pos_des     = {0.0f, 0.0f, 0.235f};
    cmd.step_height = {0.05f, 0.05f};
    motion_servo_pub_->publish(cmd);
#else
    set_velocity(x, y, yaw);   // 仿真复用旧 gamepad 接口
#endif
}
#endif

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
