// ⚠ debug_config.hpp 必须先于 motion_ctrl.hpp 包含：
//   motion_ctrl.hpp 内的 #ifdef REAL_DOG 块（protocol 发布器声明/成员）依赖 REAL_DOG 宏，
//   若后包含则 REAL_DOG 未定义，声明/实现被跳过 → 链接错误 undefined reference
#include "cyberdog_race/debug_config.hpp"
#include "cyberdog_race/motion_ctrl.hpp"
#include "cyberdog_race/control_parameter_request_lcmt.hpp"
#include "cyberdog_race/control_parameter_respones_lcmt.hpp"
#include "cyberdog_race/file_send_lcmt.hpp"
#include "cyberdog_race/motion_control_request_lcmt.hpp"
#include <cstring>
#include <thread>

MotionCtrl::MotionCtrl() : ctrl_lcm_("udpm://239.255.76.67:7671?ttl=255"),
                          param_lcm_("udpm://239.255.76.67:7668?ttl=255") {
    memset(&gpad_, 0, sizeof(gpad_));
    // 订阅参数设置响应（官方 cyberdog_app 同款：7668 上 interface_response）
    param_lcm_.subscribe("interface_response", &MotionCtrl::on_param_response, this);
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
// ⚠ 真机 pitch 方向实测：【正值=低头、负值=抬头】（舵机方向与直觉相反，2026-08-08 上机确认）
//    rpy_des=[roll,pitch,yaw]，pitch 正值=低头、负值=抬头(限 -0.25~+0.30)
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

// ── 下发身躯参数走 LCM interface_request 通道 ──
// ⚠ 2026-08-08 研究确认：真机参数通道在【端口 7668】（不是 7671！）
//   - 官方 MiRoboticsLab/cyberdog_ros2 cyberdog_app.cpp getLcmUrl(): "udpm://239.255.76.67:7668"
//     → offset_request->publish("interface_request", &request_)（control_parameter_request_lcmt）
//   - RT 板运控 hardware_bridge.cpp:64 interface_lcm_r_(7668).subscribe("interface_request", ...)
//     requestKind=3(kSET_USER_PARAM_BY_NAME) → user_control_parameters_->collection_.LookUp(name).Set()
//   - des_roll_pitch_height 正是 user param → 改它 = 走路时 roll/pitch/身高 全部可调！
// 消息: name[64] + requestNumber(单调递增) + value[96](3个double LE) + parameterKind=3 + requestKind=3
void MotionCtrl::set_body_params_lcm(float roll, float pitch, float height) {
    double v[3] = { (double)roll, (double)pitch, (double)height };
    set_user_param_lcm("des_roll_pitch_height", 3, v, 3);   // kVEC_X_DOUBLE
}

// ── 通用 user param 设置（LCM 7668 interface_request） ──
// kind: 1=kDOUBLE(value 前8字节一个double), 3=kVEC_X_DOUBLE(前24字节 n 个double)
void MotionCtrl::set_user_param_lcm(const char* name, int8_t kind, const double* vals, int n) {
    control_parameter_request_lcmt msg;
    memset(&msg, 0, sizeof(msg));
    strncpy((char*)msg.name, name, 63);
    msg.requestNumber = ++param_seq_;           // 单调递增（每次 +1）
    msg.parameterKind = kind;                   // 1=kDOUBLE 3=kVEC_X_DOUBLE
    msg.requestKind   = 3;                      // kSET_USER_PARAM_BY_NAME
    memcpy(msg.value, vals, n * sizeof(double));
    param_lcm_.publish("interface_request", &msg);   // ★ 7668 端口（官方 cyberdog_app 同款）
    // 收一下响应确认（非阻塞）
    for (int i = 0; i < 50; i++) {
        param_lcm_.handleTimeout(1);
        if (param_resp_received_) break;
    }
    fprintf(stderr, "[MotionCtrl] set_user_param_lcm(7668) %s kind=%d n=%d → %s\n",
            msg.name, (int)kind, n, param_resp_received_ ? "RT板ACK" : "无响应");
    param_resp_received_ = false;
}

void MotionCtrl::set_user_param_double_lcm(const char* name, double val) {
    double v[1] = { val };
    set_user_param_lcm(name, 1, v, 1);   // kDOUBLE
}

// ── interface_response 响应回调：确认 RT 板已设置参数 ──
void MotionCtrl::on_param_response(const lcm::ReceiveBuffer*, const std::string&,
                                   const control_parameter_respones_lcmt* msg) {
    param_resp_received_ = true;
    fprintf(stderr, "[MotionCtrl] interface_response ACK: name=%s seq=%lld kind=%d reqkind=%d\n",
            (const char*)msg->name, (long long)msg->requestNumber,
            (int)msg->parameterKind, (int)msg->requestKind);
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

// ── 真机带自定义身高行走（303 + pos_des[2]=height） ──
// ⚠ 2026-08-08 test21 发现：des_roll_pitch_height[2] 参数真机不用（体高反馈不动），
//   真机走路身高疑似走 303 命令 pos_des[2]（body_height 反馈≈pos_des[2]=0.235）
void MotionCtrl::set_walk_velocity_height(float x, float y, float yaw, float height) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_walk_velocity_height: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 303;   // MotionID::WALK_USERTROT
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {x, y, yaw};
    cmd.rpy_des     = {0.0f, 0.0f, 0.0f};
    cmd.pos_des     = {0.0f, 0.0f, height};   // 身高
    cmd.step_height = {0.15f, 0.15f};
    motion_servo_pub_->publish(cmd);
#else
    set_velocity(x, y, yaw);
#endif
}

// ── 真机静态姿态带身高（201 + pos_des[2]=height） ──
// 诊断身高通道：201 是姿态模式（rpy_des 全可用），若 pos_des[2] 生效则站姿身高变化
void MotionCtrl::set_body_pose_height(float height) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_body_pose_height: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 201;   // MotionID::FORCECONTROL_DEFINITIVELY
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {0.0f, 0.0f, 0.0f};
    cmd.rpy_des     = {0.0f, 0.0f, 0.0f};
    cmd.pos_des     = {0.0f, 0.0f, height};   // 身高
    cmd.step_height = {0.05f, 0.05f};
    motion_servo_pub_->publish(cmd);
#else
    (void)height;
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

// ── 真机姿态控制带速度+自定义身高（201 FORCECONTROL + vel_des + pos_des[2]=height） ──
// ★ 2026-08-09 路线11：命令级低姿方案！201 模式身高读 pos_des[2]（2026-08-08 实测精确控身高）
//   + vel_des 带速度 → 若能走路 = 命令级身高（不写参数内存！绝对安全！且实时可变！）
//   201 是 FORCECONTROL 姿态模式，理论上可带速度位移；需 ~20Hz 持续发布
void MotionCtrl::set_body_velocity_height(float x, float y, float yaw, float height) {
#ifdef REAL_DOG
    if (!motion_servo_pub_) {
        fprintf(stderr, "[MotionCtrl] set_body_velocity_height: 发布器未挂载(attach_motion_servo_pub)\n");
        return;
    }
    protocol::msg::MotionServoCmd cmd;
    cmd.motion_id   = 201;   // MotionID::FORCECONTROL_DEFINITIVELY
    cmd.cmd_type    = 1;     // SERVO_DATA
    cmd.cmd_source  = -1;    // DEBUG 最高优先级
    cmd.value       = 0;
    cmd.vel_des     = {x, y, yaw};          // 带速度
    cmd.rpy_des     = {0.0f, 0.0f, 0.0f};
    cmd.pos_des     = {0.0f, 0.0f, height}; // 命令级身高
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

// ── 真机节能低姿走路（7671 robot_control_cmd: mode=11 + gait_id=3 + value&0x02） ──
// ★ 2026-08-09 路线3（走路身高破解）：
//   真机 303 → LocoGaits → 身高锁 des_roll_pitch_height[2] 参数(0.25)，命令 pos_des[2] 不读，
//   且运行中改该参数[2] 会把 RT 板写崩（test20 实锤）→ 死路。
//   kUserGait(80) 走 MotionGaits 生效（身高基准 0.225 实测✓）但 pos_des[2] 偏移没生效 + 芭蕾步走不动。
//   官方留的走路低姿开关 = fsm_state_locomotion.cpp:374 LocoGaits 分支：
//     cmd_source==kCyberdog2LcmCmd(10)（7671 robot_control_cmd 自动设置）时 value&0x02
//     → use_energy_saving_mode=0 → LocoGaits 身高: use_energy_saving_mode<0.9 → pos_des_(2)=pos_cmd_min_(2)=0.13
//   → 正常 trot 前进 + 身高强制 0.13！只发命令不写参数内存 → 安全
// ⚠ 需 ~20Hz 持续发布（life_count++ 已做）
void MotionCtrl::set_walk_energy_save(float x, float y, float yaw, bool energy) {
#ifdef REAL_DOG
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode        = static_cast<int8_t>(LocoMode::LOCOMOTION);  // 11 = kLocomotion
    lcm_cmd_.gait_id     = 3;     // kTrotMedium（LocoGaits 正常前进）
    lcm_cmd_.vel_des[0]  = x;
    lcm_cmd_.vel_des[1]  = y;
    lcm_cmd_.vel_des[2]  = yaw;
    lcm_cmd_.value       = energy ? 0x02 : 0x00;   // bit1 → use_energy_saving_mode=0 → 身高强制0.13
    lcm_cmd_.step_height[0] = 150150.0f;    // 打包毫米 0.15m（LocoGaits 也是 (int)%1000 解码）
    lcm_cmd_.step_height[1] = 150150.0f;
    lcm_cmd_.life_count  = ++lcm_life_;     // 每次 +1 生效
    ctrl_lcm_.publish("robot_control_cmd", &lcm_cmd_);
#else
    (void)x; (void)y; (void)yaw; (void)energy;
#endif
}

// ── 真机 kWalkWave 走路（7671 robot_control_cmd: mode=11 + gait_id=60） ──
// ★ 2026-08-09 路线4：kWalkWave 是 MotionGaits 里能正常前进的步态（官方 walk_wave.toml 用它走路）
//   身高 = des_roll_pitch_height_motion[2](0.225) + walk_wave_height[0](0.02) ≈ 0.245
//   配合 set_body_params_motion_lcm 运行时改 des_roll_pitch_height_motion[2] → 走路中动态变身高！
//   （kUserGait00 是芭蕾走不动，改用 kWalkWave）
// ⚠ 需 ~20Hz 持续发布（life_count++ 已做）
void MotionCtrl::set_walk_wave(float x, float y, float yaw) {
#ifdef REAL_DOG
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode        = static_cast<int8_t>(LocoMode::LOCOMOTION);  // 11 = kLocomotion
    lcm_cmd_.gait_id     = 60;    // kWalkWave（MotionGaits，SetWalkWaveParams 身高=0.225+0.02）
    lcm_cmd_.vel_des[0]  = x;
    lcm_cmd_.vel_des[1]  = y;
    lcm_cmd_.vel_des[2]  = yaw;
    lcm_cmd_.step_height[0] = 150150.0f;    // 打包毫米 0.15m
    lcm_cmd_.step_height[1] = 150150.0f;
    lcm_cmd_.life_count  = ++lcm_life_;     // 每次 +1 生效
    ctrl_lcm_.publish("robot_control_cmd", &lcm_cmd_);
#else
    (void)x; (void)y; (void)yaw;
#endif
}

// ── 真机 kTrotInOut 走路（7671 robot_control_cmd: mode=11 + gait_id=56） ──
// ★ 2026-08-09 路线7：kTrotInOut = MotionGaits 里的【对角 trot 本体】！
//   control_flags_release.hpp: kTrotInOut=56；convex_mpc_motion_gaits.cpp:
//   - SetGaitParams(56) → SetTrotInOutParams: gait_type_=&trot_in_out_，offset=(0,9,9,0) 对角同相位
//     （FL+RR 同步、FR+RL 同步 = 普通 trot！），且【不清零 vel_cmd_】→ 响应速度前进
//   - SetDefaultParams: pos_cmd_[2] = des_roll_pitch_height_motion[2] → 身高走 MotionGaits 参数
//   - SetCmd: pos_des(2) 低通滤波过渡 → 走路中 7668 改 motion 参数 = 平滑动态降身高（不崩！）
//   - 前进速度上限 vel_xy_yaw_max_motion_default=[1.5,0.3,1.0]（比普通 trot 的 1.2 还快）
//   - 唯一花哨点 trot_in_out_landing_offset[0.05,0.06] 内外八（可改 RT 板 yaml 为0消除）
// ⚠ 需 ~20Hz 持续发布（life_count++ 已做）
void MotionCtrl::set_walk_trot_in_out(float x, float y, float yaw) {
#ifdef REAL_DOG
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode        = static_cast<int8_t>(LocoMode::LOCOMOTION);  // 11 = kLocomotion
    lcm_cmd_.gait_id     = 56;    // kTrotInOut（MotionGaits 对角 trot，身高=des_roll_pitch_height_motion[2]）
    lcm_cmd_.vel_des[0]  = x;
    lcm_cmd_.vel_des[1]  = y;
    lcm_cmd_.vel_des[2]  = yaw;
    lcm_cmd_.step_height[0] = 150150.0f;    // 打包毫米 0.15m
    lcm_cmd_.step_height[1] = 150150.0f;
    lcm_cmd_.life_count  = ++lcm_life_;     // 每次 +1 生效
    ctrl_lcm_.publish("robot_control_cmd", &lcm_cmd_);
#else
    (void)x; (void)y; (void)yaw;
#endif
}

// ── 真机 robot_control_cmd 普通trot + pos_des[2]（命令级身高） ──
// ★ 2026-08-09 路线15：能走的通道(robot_control_cmd gait=3) + pos_des[2]=height
//   纯命令【不发参数内存】→ 绝对安全不崩！
//   若真机固件 LocoGaits 读 ctrl_cmd_->pos_des[2] → 走路中改 = 运行时可变身高！
//   （cyberdog_sim 源码 LocoGaits 读 des_roll_pitch_height 参数，但真机固件 2025-08-01 可能不同）
void MotionCtrl::set_walk_trot_height(float x, float y, float yaw, float height) {
#ifdef REAL_DOG
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode        = static_cast<int8_t>(LocoMode::LOCOMOTION);  // 11 = kLocomotion
    lcm_cmd_.gait_id     = 3;     // kTrotMedium（普通 trot）
    lcm_cmd_.vel_des[0]  = x;
    lcm_cmd_.vel_des[1]  = y;
    lcm_cmd_.vel_des[2]  = yaw;
    lcm_cmd_.pos_des[2]  = height;    // 命令级身高
    lcm_cmd_.step_height[0] = 150150.0f;    // 打包毫米 0.15m
    lcm_cmd_.step_height[1] = 150150.0f;
    lcm_cmd_.life_count  = ++lcm_life_;     // 每次 +1 生效
    ctrl_lcm_.publish("robot_control_cmd", &lcm_cmd_);
#else
    (void)x; (void)y; (void)yaw; (void)height;
#endif
}

// ── 真机 exec_request 走路 + body_height（CyberdogLcm2Cmd 官方通道） ──
// ★ 2026-08-09 路线14：exec_request(motion_control_request_lcmt) pattern=7 → kTrotMedium(普通trot)
//   command_interface.cpp CyberdogLcm2Cmd: pattern→gait, body_height→cmd_cur_.pos_des[2]
//   【命令级身高】！不写参数内存 → 绝对安全！若真机固件 LocoGaits 读 pos_des[2]
//   → 走路中改 body_height = 运行时可变身高（官方通道）！
//   ⚠ cyberdog_sim 源码注释"body_height: donot used in locomotion"，但真机固件 2025-08-01 可能不同
//   ⚠ exec_request 是官方 NX motion_manager→RT 板的运控通道，需 ~20Hz 持续发
void MotionCtrl::set_walk_exec_request_height(float x, float y, float yaw, float height) {
#ifdef REAL_DOG
    motion_control_request_lcmt req;
    memset(&req, 0, sizeof(req));
    req.pattern     = 7;          // Pattern2Mode: 5~12→kLocomotion, 7→kTrotMedium(普通trot)
    req.linear[0]   = x;
    req.linear[1]   = y;
    req.angular[2]  = yaw;
    req.body_height = height;     // 身高（命令级！）
    req.gait_height = 0.05;       // 步高
    req.order       = 0;
    ctrl_lcm_.publish("exec_request", &req);
#else
    (void)x; (void)y; (void)yaw; (void)height;
#endif
}

// ── 运行时改 MotionGaits 身高参数 des_roll_pitch_height_motion（LCM 7668） ──
// ★ 2026-08-09 路线4：与崩 RT 板的 des_roll_pitch_height（LocoGaits 用）是【不同参数】！
//   kUserGait A 段实测真机读取 motion 参数（身高0.228≈0.225 生效）
//   → 走路中改它 = 动态变身高（kWalkWave 身高 = motion[2] + 0.02，要0.16 → motion[2]=0.14）
void MotionCtrl::set_body_params_motion_lcm(float roll, float pitch, float height) {
    double v[3] = { (double)roll, (double)pitch, (double)height };
    set_user_param_lcm("des_roll_pitch_height_motion", 3, v, 3);   // kVEC_X_DOUBLE
}

// ── 真机上传自定义步态（7671 user_gait_file 通道） ──
// ★ 2026-08-09 路线6（自定义步态）: 官方 ProcessUserGaitFile 接收 "# Gait Def" 开头的时序
//   格式（参考 user_gait_00.toml）: 每 [[section]] 的 contact=[FL,FR,RL,RR](1着地0抬起) + duration=MPC步数
//   最后一段会被强制全着地（安全）。上传后 user_gait_file_ 缓存，发 gait_id=110 时生效。
// ⚠ 需在发 kUserGait(110) 之前调用
void MotionCtrl::upload_user_gait(const std::string& gait_content) {
#ifdef REAL_DOG
    if (gait_content.empty()) return;
    file_send_lcmt msg;
    msg.data = gait_content;
    // user_gait_file 通道在 7671（hardware_bridge.cpp:66 订阅）
    lcm::LCM gait_lcm("udpm://239.255.76.67:7671?ttl=255");
    gait_lcm.publish("user_gait_file", &msg);
    // 发多次确保收到
    for (int i = 0; i < 5; i++) {
        gait_lcm.publish("user_gait_file", &msg);
        gait_lcm.handleTimeout(10);
    }
    fprintf(stderr, "[MotionCtrl] upload_user_gait(7671) %zu 字节: %.60s...\n",
            gait_content.size(), gait_content.c_str());
#else
    (void)gait_content;
#endif
}

// ── 真机 kUserGait(110) 走路（7671 robot_control_cmd） ──
// ★ 2026-08-09 路线6：配合 upload_user_gait 先上传自定义步态，再发 gait_id=110
//   fsm_state_locomotion.cpp:462 gait_id 变 kUserGait → InitUserGaits(true) 用上传步态
//   → MotionGaits SetUserGait: 身高 = des_roll_pitch_height_motion[2] + ctrl_cmd_->pos_des[2]
//   height_offset = 目标身高 - 0.225（要0.16 → -0.065）；clamp [0.16, 0.295]
// ⚠ 若没先 upload_user_gait，user_gait_file_ 为空 → InitGait 空字符串可能失败/崩（必须先上传）
void MotionCtrl::set_walk_user_gait_v2(float x, float y, float yaw, float height_offset) {
#ifdef REAL_DOG
    memset(&lcm_cmd_, 0, sizeof(lcm_cmd_));
    lcm_cmd_.mode        = static_cast<int8_t>(LocoMode::LOCOMOTION);  // 11 = kLocomotion
    lcm_cmd_.gait_id     = 110;   // kUserGait（MotionGaits，SetUserGait 身高=0.225+pos_des[2]）
    lcm_cmd_.vel_des[0]  = x;
    lcm_cmd_.vel_des[1]  = y;
    lcm_cmd_.vel_des[2]  = yaw;
    lcm_cmd_.pos_des[2]  = height_offset;   // 相对 des_roll_pitch_height_motion[2] 的偏移
    lcm_cmd_.step_height[0] = 150150.0f;    // 打包毫米 0.15m
    lcm_cmd_.step_height[1] = 150150.0f;
    lcm_cmd_.life_count  = ++lcm_life_;     // 每次 +1 生效
    ctrl_lcm_.publish("robot_control_cmd", &lcm_cmd_);
#else
    (void)x; (void)y; (void)yaw; (void)height_offset;
#endif
}
