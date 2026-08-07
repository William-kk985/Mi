#pragma once
#include <lcm/lcm-cpp.hpp>
#include "cyberdog_race/gamepad_lcmt.hpp"
#include "robot_control_cmd_lcmt.hpp"

#ifdef REAL_DOG
#include <rclcpp/rclcpp.hpp>
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
    // 真机行走/原地踏步（CyberDog2 官方接口: motion_servo_cmd + WALK_USERTROT=303）
    // x=前后 y=左右 yaw=转向 (m/s, rad/s)；全 0 = 原地踏步；需 ~20Hz 持续发布保持
    // 仿真：回退旧 gamepad set_velocity
    void set_walk_velocity(float x, float y, float yaw);
    // 分别设置左右侧步高（单位：m），step_height[0]=前/左侧，step_height[1]=后/右侧
    void set_step_height(float left, float right);
    // 真机官方跳跃（MotionResultCmd 服务，档位固定）：dist<=0.3→30cm(133)，否则 60cm(132)
    void jump_forward(float dist);

#ifdef REAL_DOG
    // 挂载 CyberDog2 motion_servo_cmd 发布器（RaceController 构造中调用）
    void attach_motion_servo_pub(rclcpp::Node* node);
    // 挂载 MotionResultCmd 服务客户端（跳跃/站立/趴下官方动作）
    void attach_motion_result_client(rclcpp::Node* node);
#endif

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
    gamepad_lcmt          gpad_;
    robot_control_cmd_lcmt lcm_cmd_;
    int                   lcm_life_{0};
    void pub_gamepad();
    void pub_lcm_cmd();

#ifdef REAL_DOG
    // CyberDog2 官方姿态控制发布器（motion_servo_cmd, motion_id=201）
    rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr motion_servo_pub_;
    // MotionResultCmd 服务客户端（跳跃/站立/趴下官方动作）
    rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_result_client_;
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
