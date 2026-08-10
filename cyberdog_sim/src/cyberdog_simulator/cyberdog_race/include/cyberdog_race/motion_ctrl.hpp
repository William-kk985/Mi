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
    // 俯仰角：⚠ 真机约定【正值=低头、负值=抬头】（舵机方向与直觉相反，2026-08-08 上机确认）
    void set_pitch(float pitch);
    // 真机姿态控制（CyberDog2 官方接口: motion_servo_cmd + FORCECONTROL_DEFINITIVELY=201）
    // ⚠ 真机约定【正值=低头、负值=抬头】（舵机方向相反），官方限 -0.25 ~ +0.30 rad
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
    // 真机带自定义身高行走（303 WALK + pos_des[2]=height）：降低/升高身体高度
    // ⚠ 2026-08-08 test21 发现：真机走路身高走 303 命令的 pos_des[2]（≈body_height 反馈），
    //   des_roll_pitch_height[2] 参数 ACK 了但真机 locomotion 似乎不用（体高反馈不动）
    void set_walk_velocity_height(float x, float y, float yaw, float height);
    // 真机静态姿态带身高（201 FORCECONTROL + pos_des[2]=height）：不移动，站姿升降（诊断身高通道用）
    void set_body_pose_height(float height);
    // 真机带自定义步高行走（303 WALK_USERTROT + step_height={h,h}）：观察抬腿高低变化
    // ⚠ 真机步高正确接口是 motion_servo_cmd.step_height 字段（旧 set_step_height 走 LCM 真机不吃）
    void set_walk_velocity_step(float x, float y, float yaw, float step_h);
    // 真机节能低姿走路（7671 robot_control_cmd: mode=11 + gait_id=3/kTrotMedium + value&0x02）
    // ★ 2026-08-09 路线3：fsm_state_locomotion.cpp:374 LocoGaits 分支
    //   cmd_source==kCyberdog2LcmCmd(10)（7671 robot_control_cmd 自动）时 value&0x02
    //   → use_energy_saving_mode=0 → LocoGaits 身高 pos_des_(2)=pos_cmd_min_(2)=0.13（强制最低）
    //   → 正常 trot 前进 + 身高强制 0.13！只发命令不写参数内存（不会崩 RT 板）
    //   energy=true 发 value=0x02，false 发 value=0（对照）
    void set_walk_energy_save(float x, float y, float yaw, bool energy);
    // 真机 kWalkWave 走路（7671 robot_control_cmd: mode=11 + gait_id=60）——MotionGaits 能走步态
    // ★ 2026-08-09 路线4：kWalkWave 身高 = des_roll_pitch_height_motion[2] + walk_wave_height
    //   （convex_mpc_motion_gaits.cpp:SetWalkWaveParams）→ 配合 set_body_params_motion_lcm 走路中动态变身高
    //   kUserGait00 是芭蕾走不动，kWalkWave(60) 是官方 walk_wave.toml 走路动作（能走）
    void set_walk_wave(float x, float y, float yaw);
    // 真机 exec_request 走路 + body_height（7671 exec_request 通道，CyberdogLcm2Cmd）
    // ★ 2026-08-09 路线14：pattern=7→kTrotMedium(普通trot)，body_height→cmd_cur_.pos_des[2]
    //   【命令级身高】不写参数内存→绝对安全！若真机固件 LocoGaits 读 pos_des[2]
    //   → 走路中改 body_height = 运行时可变身高（官方 NX→RT 通道）！
    void set_walk_exec_request_height(float x, float y, float yaw, float height);
    // 真机 robot_control_cmd 普通trot + pos_des[2]（命令级身高）
    // ★ 2026-08-09 路线15：能走的通道(robot_control_cmd gait=3) + pos_des[2]=height
    //   纯命令不发参数→绝对安全！若真机固件 LocoGaits 读 pos_des[2]
    //   → 走路中改 pos_des[2] = 运行时可变身高（最后一个命令级候选）
    void set_walk_trot_height(float x, float y, float yaw, float height);
    // 真机 kTrotInOut 走路（7671 robot_control_cmd: mode=11 + gait_id=56）——MotionGaits 里的对角 trot 本体！
    // ★ 2026-08-09 路线7：control_flags_release.hpp kTrotInOut=56，convex_mpc_motion_gaits.cpp SetTrotInOutParams
    //   gait_type_=&trot_in_out_，offset=(0,9,9,0) 对角同相位=普通 trot，且不清零 vel_cmd_ → 响应速度前进（上限1.5m/s）
    //   身高 = des_roll_pitch_height_motion[2]（SetDefaultParams）→ 配合 set_body_params_motion_lcm 走路中动态变身高！
    //   唯一花哨点 trot_in_out_landing_offset[0.05,0.06] 内外八（可改 RT 板 yaml 为0消除）
    void set_walk_trot_in_out(float x, float y, float yaw);
    // 运行时改 MotionGaits 身高参数 des_roll_pitch_height_motion（LCM 7668）
    // ★ 2026-08-09 路线4：与崩 RT 板的 des_roll_pitch_height（LocoGaits 用）是不同参数！
    //   kUserGait A 段实测 motion 参数被真机读取（0.225 生效）→ 改它 = 走路中动态变身高
    // ⚠ 同样走 7668 参数通道，若崩需重启狗（风险比 des_roll_pitch_height[2] 低，但未实测）
    void set_body_params_motion_lcm(float roll, float pitch, float height);
    // 真机上传自定义步态（7671 user_gait_file 通道，官方 ProcessUserGaitFile）
    // ★ 2026-08-09 路线6：官方留的自定义步态通道！内容 "# Gait Def" 开头 + contact/duration 时序
    //   → user_gait_file_ 缓存 → 发 gait_id=110(kUserGait) 时 InitUserGaits(true) 用它初始化
    //   → 自定义普通 trot 时序 + kUserGait 走路 + motion 参数控身高（三步全通！）
    void upload_user_gait(const std::string& gait_content);
    // 真机 kUserGait(110) 走路（7671 robot_control_cmd mode=11 + gait_id=110）
    // ★ 2026-08-09 路线6：配合 upload_user_gait 先上传自定义步态，再发 110 走路
    //   fsm_state_locomotion.cpp:462 gait_id 变 110 → InitUserGaits(true) 用上传步态
    //   → MotionGaits SetUserGait 身高 = des_roll_pitch_height_motion[2] + pos_des[2]
    void set_walk_user_gait_v2(float x, float y, float yaw, float height_offset);
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
    // 真机姿态控制带速度+自定义身高（201 FORCECONTROL + vel_des + pos_des[2]=height）
    // ★ 2026-08-09 路线11：命令级方案！201 模式身高读 pos_des[2]（实测精确控身高）
    //   + vel_des 带速度 → 若能走路 = 命令级身高（不写参数内存，绝对安全，且实时可变！）
    void set_body_velocity_height(float x, float y, float yaw, float height);
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
    // 通用 user param 设置（LCM 7668 interface_request）：kind=1(kDOUBLE)/3(kVEC_X_DOUBLE)，vals 传 n 个
    void set_user_param_lcm(const char* name, int8_t kind, const double* vals, int n);
    // 便捷：设置单个 double user param（如 x_effect_scale_pos 破 pitch 限位）
    void set_user_param_double_lcm(const char* name, double val);

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
