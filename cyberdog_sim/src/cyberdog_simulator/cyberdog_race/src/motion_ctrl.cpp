// ⚠ debug_config.hpp 必须先于 motion_ctrl.hpp 包含：
//   motion_ctrl.hpp 内的 #ifdef REAL_DOG 块（protocol 发布器声明/成员）依赖 REAL_DOG 宏，
//   若后包含则 REAL_DOG 未定义，声明/实现被跳过 → 链接错误 undefined reference
#include "cyberdog_race/debug_config.hpp"
#include "cyberdog_race/motion_ctrl.hpp"
#include <cstring>
#include <thread>

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

void MotionCtrl::attach_motion_result_client(rclcpp::Node* node) {
    motion_result_client_ = node->create_client<protocol::srv::MotionResultCmd>(
        ROBOT_NS "/motion_result_cmd");
    RCLCPP_INFO(node->get_logger(), "[MotionCtrl] motion_result_cmd 客户端已挂载(%s)",
                ROBOT_NS "/motion_result_cmd");
}

// ── 真机官方动作触发（MotionResultCmd 服务） ──
//   motion_id: 111=站立 101=趴下 133=前跳30cm 132=前跳60cm ...
void MotionCtrl::send_result_cmd(int motion_id) {
    if (!motion_result_client_) {
        fprintf(stderr, "[MotionCtrl] send_result_cmd(%d): 客户端未挂载\n", motion_id);
        return;
    }
    if (!motion_result_client_->service_is_ready()) {
        fprintf(stderr, "[MotionCtrl] send_result_cmd(%d): 服务未就绪\n", motion_id);
        return;
    }
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    req->motion_id = motion_id;
    auto future = motion_result_client_->async_send_request(req);
    fprintf(stderr, "[MotionCtrl] result_cmd motion_id=%d\n", motion_id);
}
#endif

// ── 真机官方跳跃（motion_servo_cmd 持续发布，档位固定） ──
//   ⚠ 跳跃不走 MotionResultCmd（Command 133 not valid, 2026-08-07 上机确认）
//   与姿态201/行走303一样走 ServoCmd，持续发布 ~1.5s 触发跳跃轨迹
//   motion_id: 133=前跳30cm, 132=前跳60cm（MotionID 预设轨迹，无法自定义距离）
void MotionCtrl::jump_forward(float dist) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] jump_forward: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = (dist <= 0.3f) ? 133 : 132;   // 前跳30cm / 60cm
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {0.0f, 0.0f, 0.0f};
    cmd.rpy_des     = {0.0f, 0.0f, 0.0f};
    cmd.pos_des     = {0.0f, 0.0f, 0.235f};
    cmd.step_height = {0.05f, 0.05f};
    // 跳跃是 ServoCmd 方式：持续发布 ~1.5s 让 motion_manager 执行跳跃轨迹
    for (int i = 0; i < 30; i++) {   // 30帧 @ 50ms = 1.5s
        motion_servo_pub_->publish(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    fprintf(stderr, "[MotionCtrl] jump_forward(ServoCmd): motion_id=%d (%.0fcm)\n",
            cmd.motion_id, dist * 100);
#else
    // 仿真：旧 LCM JUMP_3D
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode       = static_cast<int8_t>(LocoMode::JUMP_3D);
    lcm_cmd_.gait_id    = 1;
    lcm_cmd_.vel_des[0] = 0.4f;
    pub_lcm_cmd();
#endif
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
    cmd.step_height = {0.15f, 0.15f};   // 步高(2026-08-07 调高测试: 0.05→0.10→0.15)
    motion_servo_pub_->publish(cmd);
#else
    set_velocity(x, y, yaw);   // 仿真复用旧 gamepad 接口
#endif
}

// ── 真机带俯仰姿态行走（303 + rpy_des[1]=pitch） ──
// ⚠ 官方 motion_teleop 只设 vel_des；此处尝试行走时同时保持机身俯仰（抬头/低头）
//   需 ~20Hz 持续发布。能否生效上机验证（2026-08-08）
void MotionCtrl::set_walk_velocity_pitch(float x, float y, float yaw, float pitch) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_walk_velocity_pitch: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 303;   // MotionID::WALK_USERTROT
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {x, y, yaw};
    cmd.rpy_des     = {0.0f, pitch, 0.0f};   // 带俯仰
    cmd.pos_des     = {0.0f, 0.0f, 0.235f};
    cmd.step_height = {0.15f, 0.15f};
    motion_servo_pub_->publish(cmd);
#else
    set_velocity(x, y, yaw);
#endif
}

// ── 真机姿态控制同时带速度（201 + vel_des） ──
// ⚠ 官方 pose_teleop 只设 rpy_des(vel_des=0)；此处尝试姿态模式直接带速度行走
//   需 ~20Hz 持续发布。能否生效上机验证（2026-08-08）
void MotionCtrl::set_body_pitch_velocity(float pitch, float x, float y, float yaw) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_body_pitch_velocity: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 201;   // MotionID::FORCECONTROL_DEFINITIVELY
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {x, y, yaw};          // 带速度
    cmd.rpy_des     = {0.0f, pitch, 0.0f};
    cmd.pos_des     = {0.0f, 0.0f, 0.235f};
    cmd.step_height = {0.15f, 0.15f};
    motion_servo_pub_->publish(cmd);
#else
    set_velocity(x, y, yaw);
#endif
}

// ⚠ 真机站/趴必须走 MotionResultCmd 服务（gamepad 是铁蛋一代接口，真机无效——2026-08-07 踩坑）
void MotionCtrl::stand() {
#ifdef REAL_DOG
    send_result_cmd(111);   // RECOVERYSTAND
#else
    gpad_.x = 1; pub_gamepad(); gpad_.x = 0;
#endif
}
void MotionCtrl::locomotion() { gpad_.y = 1; pub_gamepad(); gpad_.y = 0; }
void MotionCtrl::lie_down() {
#ifdef REAL_DOG
    send_result_cmd(101);   // GETDOWN
#else
    gpad_.a = 1; pub_gamepad(); gpad_.a = 0;
#endif
}
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
