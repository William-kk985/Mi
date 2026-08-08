#pragma once
#include <lcm/lcm-cpp.hpp>
#include "cyberdog_race/gamepad_lcmt.hpp"
#include "robot_control_cmd_lcmt.hpp"
#include "cyberdog_race/control_parameter_request_lcmt.hpp"
#include "cyberdog_race/control_parameter_respones_lcmt.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cyberdog_msg/msg/yaml_param.hpp>
#ifdef REAL_DOG
#include <protocol/msg/motion_servo_cmd.hpp>
#include <protocol/srv/motion_result_cmd.hpp>
#endif

// robot_control_cmd.mode 枚举（官方文档 cyberdog_loco_cn.md §2.1）
enum class LocoMode : int8_t {
    PURE_DAMPER   = 0,   // 纯阻尼
    LOCOMOTION    = 11,  // 行走模式（trot）
    STAND         = 12,  // QP站立
    JUMP_3D       = 16,  // 离线轨迹跳跃
    POSE_CTRL     = 21,  // 位控姿态模式（rpy_des 控制 roll/pitch/yaw）
    FORCE_JUMP    = 22,  // 力控跳跃
};

class MotionCtrl {
public:
    MotionCtrl();

    // 速度控制：x前后，y左右，yaw转向（正=左转，负=右转）
    void set_velocity(float x, float y, float yaw);
    // 俯仰角：负值低头，正值抬头
    void set_pitch(float pitch);
    // 真机姿态控制（CyberDog2 官方接口: motion_servo_cmd + FORCECONTROL_DEFINITIVELY=201）
    // pitch 负值低头、正值抬头，官方限 -0.25 ~ +0.30 rad
    // 注意：需 attach_motion_servo_pub() 挂载发布器后生效；需以 ~20Hz 持续调用保持
    void set_body_pitch(float pitch);
    // 真机机身 yaw 姿态控制（201 FORCECONTROL + rpy_des[2]=yaw）：不移动，机身偏转朝向
    // yaw 正值=左偏(CCW)、负值=右偏(CW)，官方限 ±0.65 rad（pose_teleop a/d 键同款）
    void set_body_yaw(float yaw);
    // 真机身躯侧倾/倾斜（201 FORCECONTROL + rpy_des[0]=roll）：不移动，身体左右倾斜
    // roll 正负=左右倾，官方限 ±0.52 rad（pose_teleop j/l 键同款）
    void set_body_roll(float roll);
    // 真机行走/原地踏步（CyberDog2 官方接口: motion_servo_cmd + WALK_USERTROT=303）
    // x=前后 y=左右 yaw=转向 (m/s, rad/s)；全 0 = 原地踏步；需 ~20Hz 持续发布保持
    // 仿真：回退旧 gamepad set_velocity
    void set_walk_velocity(float x, float y, float yaw);
    // 真机带自定义步高行走（303 WALK_USERTROT + step_height={h,h}）：观察抬腿高低变化
    // ⚠ 真机步高正确接口是 motion_servo_cmd.step_height 字段（旧 set_step_height 走 LCM 真机不吃）
    void set_walk_velocity_step(float x, float y, float yaw, float step_h);
    // 步高原始值直通（不clamp）：排查编码——仿真控制器 (int)%1000*1e-3 解码，
    // 疑似期望毫米(250→0.25m)/打包(250250)而非米(0.25→(int)=0)。2026-08-08 待上机验证
    void set_walk_velocity_step_raw(float x, float y, float yaw, float step_h_raw);
    // 真机带俯仰姿态行走（303 WALK_USERTROT + rpy_des[1]=pitch）：走路同时保持抬头/低头
    // ✅ 2026-08-08 上机验证：test17 低头保持 ~-5° 走满 0.3m（步态读 rpy_des[1]）
    void set_walk_velocity_pitch(float x, float y, float yaw, float pitch);
    // 真机带 roll+pitch 姿态行走（303 WALK_USERTROT + rpy_des=[roll,pitch,0]）
    // ⚠ 官方 locomotion 只启用 pitch（rpy_cmd_scale=[0,1,0]），roll 走命令是否生效待验证（test18）
    void set_walk_velocity_rpy(float x, float y, float yaw, float roll, float pitch);
    // 真机姿态控制同时带速度（201 FORCECONTROL + vel_des）：姿态模式下直接行走
    // ⚠ 官方 pose teleop vel_des=0，能否带速度需上机验证（2026-08-08 待测）
    void set_body_pitch_velocity(float pitch, float x, float y, float yaw);
    // 真机姿态控制带速度（201 FORCECONTROL + rpy_des=[roll,pitch,0] + vel_des）：力控模式直接带姿态行走
    void set_body_rpy_velocity(float roll, float pitch, float x, float y, float yaw);
    // 分别设置左右侧步高（单位：m），step_height[0]=前/左侧，step_height[1]=后/右侧
    void set_step_height(float left, float right);
    // 真机官方跳跃（MotionResultCmd 服务，档位固定）：dist<=0.3→30cm(133)，否则 60cm(132)
    void jump_forward(float dist);
    // 真机官方动作触发（MotionResultCmd 服务）：111=站立 101=趴下 133=前跳30cm 132=前跳60cm
    void send_result_cmd(int motion_id);
    // 挂载 yaml_parameter 发布器（赛段姿态参数 des_roll_pitch_height，走路时保持姿态用）
    void attach_yaml_pub(rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr pub);
    // 下发身躯参数：roll/pitch/身高（des_roll_pitch_height, 真机走路时姿态可能靠它保持）
    void set_body_params_yaml(float roll, float pitch, float height);
    // 下发身躯参数走【LCM interface_request 通道】（control_parameter_request_lcmt, kSET_USER_PARAM_BY_NAME）
    // ⚠ 2026-08-08 发现：真机 yaml_parameter ROS topic 无订阅者（死通道），但运控有 LCM 参数通道
    //   （hardware_bridge.cpp:64 interface_lcm_r_.subscribe("interface_request", ...)
    //     → kSET_USER_PARAM_BY_NAME 分支 LookUp(name).Set()，des_roll_pitch_height 正是 user param）
    //   绕开死 ROS topic，直接改 RT 板运控 user params → 走路时 roll 保持！
    void set_body_params_lcm(float roll, float pitch, float height);

#ifdef REAL_DOG
    // 挂载 CyberDog2 motion_servo_cmd 发布器（RaceController 构造中调用）
    void attach_motion_servo_pub(rclcpp::Node* node);
    // 挂载 MotionResultCmd 服务客户端（跳跃/站立/趴下官方动作）
    void attach_motion_result_client(rclcpp::Node* node);
#endif

    // 步高原始值直通（无 clamp，走 LCM robot_control_cmd 7671）——test8 方式：起步前设一次
    // 仿真控制器解码 (int)%1000*1e-3，疑似期望毫米/打包格式；0.20米会被 (int) 截成 0
    void set_step_height_raw(float left, float right);

    // 模式切换
    void stand();        // QP站立
    void locomotion();   // 行走模式
    void lie_down();     // 趴下
    void recovery();     // 恢复站立
    void stop();         // 停止（发零速）
    void jump();         // kJump3d (mode=16), 离线轨迹跳跃
    void force_jump();   // kForceJump (mode=22), 力控跳跃
    void send_lcm_mode(int mode, int gait_id = 0);

    // TODO: 添加 robot_control_response 订阅以检查模式切换是否成功
    // 真狗在 7670 端口发布 robot_control_response（robot_control_response_lcmt）
    // 需从真狗 SDK 获取类型定义后取消注释：
    //   bool is_mode_ok() const { return mode_ok_; }
    // private:
    //   std::atomic<bool> mode_ok_{false};
    //   void on_response(const lcm::ReceiveBuffer*, const std::string&,
    //                    const robot_control_response_lcmt* msg) {
    //       mode_ok_ = true;
    //   }
    // 构造中: ctrl_lcm_.subscribe("robot_control_response", &MotionCtrl::on_response, this);

private:
    lcm::LCM              lcm_;           // 7667端口，gamepad
    lcm::LCM              ctrl_lcm_;      // 7671端口，robot_control_cmd (也用于7670订阅响应)
    lcm::LCM              param_lcm_;     // 7668端口，interface_request 控制参数通道（官方 cyberdog_app 同款）
    gamepad_lcmt          gpad_;
    robot_control_cmd_lcmt lcm_cmd_;
    int                   lcm_life_{0};
    int64_t               param_seq_{0};  // interface_request 参数请求序号（单调递增）
    bool                  param_resp_received_{false};  // interface_response 已收到
    void pub_gamepad();
    void pub_lcm_cmd();
    // interface_response 响应回调（确认 RT 板收到参数设置）
    void on_param_response(const lcm::ReceiveBuffer*, const std::string&,
                           const control_parameter_respones_lcmt* msg);

#ifdef REAL_DOG
    // CyberDog2 官方姿态控制发布器（motion_servo_cmd, motion_id=201）
    rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr motion_servo_pub_;
    // MotionResultCmd 服务客户端（跳跃/站立/趴下官方动作）
    rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_result_client_;
    // yaml_parameter 发布器（赛段姿态参数）
    rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr yaml_pub_;
#endif

    // ═══ TODO: 电机温度监控（需 danger_states_lcmt.hpp，lcm-gen -x 生成） ═══
    //   float motor_temp_[12]{};
    //   std::atomic<bool> motor_overheat_{false};
    //   void on_motor_temp(const lcm::ReceiveBuffer*, const std::string&,
    //                      const danger_states_lcmt* msg);
    // 构造中: lcm_.subscribe("motor_temperature", &MotionCtrl::on_motor_temp, this);
    // 使用:   bool is_overheat() const { return motor_overheat_; }
    //
    // void MotionCtrl::on_motor_temp(..., const danger_states_lcmt* msg) {
    //     float max_t = 0;
    //     for (int i = 0; i < 12; i++) {
    //         motor_temp_[i] = msg->motor_temperature[i];
    //         if (motor_temp_[i] > max_t) max_t = motor_temp_[i];
    //     }
    //     motor_overheat_ = (max_t > 74.f);  // 官方文档: 74°C 预警
    // }

    // ═══ TODO: robot_control_response 命令确认（需 robot_control_response_lcmt.hpp） ═══
    //   std::atomic<bool> cmd_ok_{false};
    //   std::atomic<int8_t> curr_mode_{0};
    //   void on_cmd_response(const lcm::ReceiveBuffer*, const std::string&,
    //                        const robot_control_response_lcmt* msg);
    //   bool wait_cmd_ok(int timeout_ms = 100);
    // 构造中: ctrl_lcm_.subscribe("robot_control_response", &MotionCtrl::on_cmd_response, this);
    //
    // void MotionCtrl::on_cmd_response(..., const robot_control_response_lcmt* msg) {
    //     curr_mode_ = msg->mode;
    //     cmd_ok_ = true;
    // }
    // bool MotionCtrl::wait_cmd_ok(int timeout_ms) {
    //     auto t0 = std::chrono::steady_clock::now();
    //     while (!cmd_ok_) {
    //         if (std::chrono::steady_clock::now() - t0 >
    //             std::chrono::milliseconds(timeout_ms)) return false;
    //         std::this_thread::sleep_for(std::chrono::milliseconds(5));
    //     }
    //     cmd_ok_ = false;
    //     return true;
    // }
};
