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

// ── 真机机身 yaw 姿态控制（201 + rpy_des[2]=yaw） ──
// ⚠ 与 set_body_pitch 同理，走 201 FORCECONTROL；不移动，机身偏转朝向
//    yaw 正值=左偏、负值=右偏（pose_teleop a/d 键同款，官方限 ±0.65）；需 ~20Hz 持续发布
void MotionCtrl::set_body_yaw(float yaw) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_body_yaw: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 201;   // MotionID::FORCECONTROL_DEFINITIVELY
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {0.0f, 0.0f, 0.0f};
    cmd.rpy_des     = {0.0f, 0.0f, yaw};   // 机身yaw（正=左, 负=右）
    cmd.pos_des     = {0.0f, 0.0f, 0.235f};
    cmd.step_height = {0.05f, 0.05f};
    motion_servo_pub_->publish(cmd);
#else
    // 仿真：旧 LCM mode=21 POSE_CTRL
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode       = static_cast<int8_t>(LocoMode::POSE_CTRL);  // 21
    lcm_cmd_.gait_id    = 0;
    lcm_cmd_.rpy_des[2] = yaw;   // yaw
    lcm_cmd_.duration   = 500;
    pub_lcm_cmd();
#endif
}

// ── 真机身躯侧倾/倾斜（201 + rpy_des[0]=roll） ──
// ⚠ 与 set_body_pitch/set_body_yaw 同理，走 201 FORCECONTROL；不移动，身体左右倾斜
//    roll 正负=左右倾（pose_teleop j/l 键同款，官方限 ±0.52）；需 ~20Hz 持续发布
void MotionCtrl::set_body_roll(float roll) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_body_roll: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 201;   // MotionID::FORCECONTROL_DEFINITIVELY
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {0.0f, 0.0f, 0.0f};
    cmd.rpy_des     = {roll, 0.0f, 0.0f};   // 侧倾（正负=左右）
    cmd.pos_des     = {0.0f, 0.0f, 0.235f};
    cmd.step_height = {0.05f, 0.05f};
    motion_servo_pub_->publish(cmd);
#else
    // 仿真：旧 LCM mode=21 POSE_CTRL
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode       = static_cast<int8_t>(LocoMode::POSE_CTRL);  // 21
    lcm_cmd_.gait_id    = 0;
    lcm_cmd_.rpy_des[0] = roll;   // roll
    lcm_cmd_.duration   = 500;
    pub_lcm_cmd();
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

void MotionCtrl::attach_yaml_pub(rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr pub) {
    yaml_pub_ = pub;
}

// ── 下发身躯参数（des_roll_pitch_height） ──
// ⚠ 赛段机制：真机走路时 roll/pitch/身高 靠 yaml_parameter 参数保持（非伺服命令，不会被 303 冲掉）
//   与 race_controller apply_stage_params 同款：kind=3, vecxd_value=[roll, pitch, height]
void MotionCtrl::set_body_params_yaml(float roll, float pitch, float height) {
    if (!yaml_pub_) {
        fprintf(stderr, "[MotionCtrl] set_body_params_yaml: yaml_pub_ 未挂载(attach_yaml_pub)\n");
        return;
    }
    cyberdog_msg::msg::YamlParam p;
    p.name = "des_roll_pitch_height";
    p.kind = 3; p.is_user = 1;
    p.vecxd_value[0] = roll;
    p.vecxd_value[1] = pitch;
    p.vecxd_value[2] = height;
    yaml_pub_->publish(p);
    fprintf(stderr, "[MotionCtrl] set_body_params_yaml roll=%.2f pitch=%.2f h=%.2f\n", roll, pitch, height);
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

// ── 真机带自定义步高行走（303 + step_height） ──
// ⚠ 真机步高正确接口是 motion_servo_cmd.step_height 字段（官方 303 preset 同款），
//   旧 set_step_height 走 LCM robot_control_cmd(7671) 真机不吃（2026-08-08 确认）。
//   step_h 建议范围 0.05~0.25m（官方 clamp 0~0.35）；需 ~20Hz 持续发布
void MotionCtrl::set_walk_velocity_step(float x, float y, float yaw, float step_h) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_walk_velocity_step: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    float h = step_h < 0.0f ? 0.0f : (step_h > 0.35f ? 0.35f : step_h);
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 303;   // MotionID::WALK_USERTROT
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {x, y, yaw};
    cmd.rpy_des     = {0.0f, 0.0f, 0.0f};
    cmd.pos_des     = {0.0f, 0.0f, 0.235f};
    cmd.step_height = {h, h};   // 自定义步高
    motion_servo_pub_->publish(cmd);
#else
    set_velocity(x, y, yaw);
#endif
}

// ── 步高原始值直通（不clamp，排查编码） ──
// 仿真控制器解码：step_height_cmd = (int)step_height % 1000 * 1e-3
//   → 若真机透传该字段，米(0.25)会被 (int) 截成 0，毫米(250)才能得到 0.25m
void MotionCtrl::set_walk_velocity_step_raw(float x, float y, float yaw, float step_h_raw) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_walk_velocity_step_raw: 发布器未挂载(attach_motion_servo_pub)\n");
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
    cmd.step_height = {step_h_raw, step_h_raw};   // 原始值直通
    motion_servo_pub_->publish(cmd);
#else
    set_velocity(x, y, yaw);
#endif
}

// ── 真机带俯仰姿态行走（303 + rpy_des[1]=pitch） ──
// ✅ 2026-08-08 上机验证：test17 低头保持 ~-5° 走满 0.3m
//   （步态控制器 convex_mpc_loco_gaits.cpp:2305 走路时 rpy_cmd_[1]=ctrl_cmd_->rpy_des[1]）
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

// ── 真机带 roll+pitch 姿态行走（303 + rpy_des=[roll,pitch,0]） ──
// ⚠ 官方 locomotion rpy_cmd_scale=[0,1,0]（只有 pitch 启用），roll 走命令是否生效待验证（test18）
void MotionCtrl::set_walk_velocity_rpy(float x, float y, float yaw, float roll, float pitch) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_walk_velocity_rpy: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 303;   // MotionID::WALK_USERTROT
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {x, y, yaw};
    cmd.rpy_des     = {roll, pitch, 0.0f};   // 带 roll+pitch
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

// ── 真机姿态控制带速度（201 + rpy_des=[roll,pitch,0] + vel_des） ──
// 力控模式直接带 roll 姿态 + 前进速度（test18 验证 roll 走路）
void MotionCtrl::set_body_rpy_velocity(float roll, float pitch, float x, float y, float yaw) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_body_rpy_velocity: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 201;   // MotionID::FORCECONTROL_DEFINITIVELY
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {x, y, yaw};          // 带速度
    cmd.rpy_des     = {roll, pitch, 0.0f};  // 带 roll+pitch
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

// ⚠ 2026-08-08 修复：真机步高经 LCM robot_control_cmd(7671) 生效，但控制器解码是打包毫米：
//    step_height_cmd = (int)%1000*1e-3（低3位=脚A）+ /1000*1e-3（高3位=脚B）
//    所以 0.15m → 150mm → 打包 150*1000+150 = 150150（同一元素内两只脚同高）
//    直接发米(0.15) 会被 (int) 截成 0（之前一直这样，真机没反应——2026-08-08 上机确认）
void MotionCtrl::set_step_height(float left, float right) {
    int lmm = (int)(left  * 1000.0f);  lmm = lmm < 0 ? 0 : (lmm > 300 ? 300 : lmm);
    int rmm = (int)(right * 1000.0f);  rmm = rmm < 0 ? 0 : (rmm > 300 ? 300 : rmm);
    float lpack = (float)(lmm * 1000 + lmm);   // [0]: 低3位+高3位 = left 高度（脚0,1）
    float rpack = (float)(rmm * 1000 + rmm);   // [1]: 低3位+高3位 = right 高度（脚2,3）
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.step_height[0] = lpack;
    lcm_cmd_.step_height[1] = rpack;
    ctrl_lcm_.publish("robot_control_cmd", &lcm_cmd_);  // LCM robot_control_cmd 7671
    fprintf(stderr, "[MotionCtrl] set_step_height(%.2fm → 打包%.0f,%.0f)\n", left, lpack, rpack);
}

// ── 步高原始值直通（无clamp，test8 方式：起步前设一次） ──
// ⚠ 走 LCM robot_control_cmd(7671)，README 确认真机 motion_manager 会收此通道（mode=21→303 映射）
//   仿真控制器解码 (int)%1000*1e-3 → 疑似期望毫米(200=0.2m)/打包(200200)而非米(0.2→0)
void MotionCtrl::set_step_height_raw(float left, float right) {
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.step_height[0] = left;
    lcm_cmd_.step_height[1] = right;
    ctrl_lcm_.publish("robot_control_cmd", &lcm_cmd_);
    fprintf(stderr, "[MotionCtrl] set_step_height_raw(%.0f, %.0f) → LCM robot_control_cmd\n", left, right);
}
