#pragma once

#include <chrono>
#include <cstdint>

#include "cyberdog_race/debug_config.hpp"
#include "cyberdog_race/stages/stage_base.hpp"

// Stage 4 real-track: 蛇形三通道巡逻
//
// 每轮动作：
//   前进1m → 右转90° → 前进2.8m（去程检测）→ 掉头180° → 回来2.8m（回程检测）→ 右转90°
// 共3轮（蛇形扫3条平行通道，全右转衔接）
//
// 去程2.8m内检测（全程低头）：
//   - 限高杆 <0.8m → 只播报（全程已低头，不再切低姿状态）
//   - 可乐/足球 ≤0.8m → 冲过去撞击 → 继续走到终点
// 回程2.8m内检测：只播报限高杆，不撞可乐
//
// 低姿实现（官方源码链路，convex_mpc_loco_gaits.cpp:2330）：
//   7671 robot_control_cmd value&0x02 → use_energy_saving_mode=0
//   → pos_des_(2)=pos_cmd_min_(2)=0.13（正常 trot 前进，不写参数内存，安全）
//   起立：value=0x00 持续发 → use_energy_saving_mode=1 → 身高低通回升
class Stage4Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override;

    float get_desired_height()      const override { return kStandHeight; }
    float get_desired_step_height() const override { return kStandStepH; }

protected:   // 放宽访问级别, 供派生类覆盖路线
    // 身高/步高
    static constexpr float kStandHeight = 0.24f;
    static constexpr float kStandStepH  = 0.10f;

    // ═══ 路线参数 (成员变量, 派生类 set_route_params() 覆盖换路线) ═══
    // 默认值 = 正式版; 派生类 override 即可, 无需改宏/多二进制
    static constexpr int   kMaxRounds = 3;         // 三轮
    float kFwdDist[3]      = {0.8f, 1.1f, 1.0f};    // 起步前进按轮次 (正式)
    float kLaneDist[3]     = {3.5f, 3.35f, 2.0f};   // 去程通道按轮次 (正式)
    float kLaneDistBack[3] = {3.55f, 2.45f, 2.25f}; // 回程通道按轮次 (正式)
    float kExitFwd1{1.0f};    // 离场第1段 (正式)
    float kExitFwd2{1.15f};   // 离场第2段 (正式)
    float kExitFwd3{1.8f};    // 离场第3段 (正式)
    float kFwdLatComp{0.023f};   // 前进横向补偿 (正式)
    float kFwdLatCompLow{0.020f};// 破限段横向补偿 (正式)
    // 绕行/链接段 (各路线相同, 基类内部使用)
    static constexpr float kSpecialFwd1 = 1.1f;  // 绕行: 左转90°后前进
    static constexpr float kSpecialFwd2 = 0.95f; // 绕行: 右转90°后前进
    static constexpr float kLinkFwd1 = 1.8f;     // 第1轮后前进1.8m (test2)
    static constexpr float kLinkFwd2 = 2.5f;     // 第2轮左转后前进2.5m (test2)

    // ★ 路线差异覆盖点: 派生类 override 换路线, init() 会调用 set_route_params()
    virtual void set_route_params() {}   // 默认成员初始化器已是正式版值
    virtual bool route_after_fwd1m() { return false; }    // FWD_1M 走满后: true=派生已接管流转
    virtual bool route_after_round_out() { return false; }// TURN_R_OUT: true=派生已接管衔接
    virtual bool route_after_exit_fwd1() { return false; }// 离场第1段后: true→TURN_EXIT_L2

    // 运动参数
    static constexpr float kWalkSpeed = 0.30f;
    static constexpr float kLowSpeed  = 0.20f;     // 限高杆下低姿速度
    static constexpr float kKnockSpeed= 0.35f;     // 撞击可乐/足球速度
    static constexpr float kTurnRate  = 0.35f;
    static constexpr float kYawKp     = 1.2f;   // 行走 yaw 增益 (防长距离方向右漂)
    static constexpr float kYawTol    = 0.03f;   // 转向到位误差 (太宽致转向时多时少, 对齐 Stage2 收紧)
    // “物理欠转2°”假设已证不成立, 转满 (与 Stage2 对齐)
    static constexpr float kTurnExtraRad = 0.0f;
    static constexpr float kTurnYawBias  = 0.05f;   // 转向 yaw 偏置 (正式/test2 统一)
    static constexpr float kYawDeadband = 0.005f;  // 航向死区 (方向漂移及时拉回, Stage2 同款)
    static constexpr float kTurnLatComp = -0.01f;   // 原地转向右漂: 固定向左补偿 (负=左)
    static constexpr float kTurnHoldLimit = 0.12f;  // 转向位置保持限幅

    // 任务参数
    static constexpr float kLimbarTriggerDist = 0.8f;  // 限高杆 <0.8m 触发播报
    static constexpr float kLimbarPitch       = 0.25f; // 限高杆下低头 ~14° (pitch 正值=低头)
    // 低头破限: 303 走路 pitch 硬限 ±5.7°, 大命令无效; x_effect_scale_pos=+30 放大限位才走满; 过杆后复原默认 -0.55
    static constexpr double kPitchScaleUnlock  = 30.0;
    static constexpr double kPitchScaleDefault = -0.55;
    static constexpr float kLimbarLowDist     = 1.0f;  // 低姿走1m后起立
    static constexpr float kKnockTriggerDist  = 0.8f;  // 可乐/足球≤0.8m撞击
    static constexpr float kKnockMinDist      = 0.12f; // 已贴脸(≈已撞)不再触发
    static constexpr float kKnockDist         = 0.4f;  // 撞击最多冲0.4m
    static constexpr int   kRiseFrames        = 30;    // 起立帧数(energy=false回置身高)

    // 阈值与超时
    static constexpr int   kReadyFrames   = 5;
    static constexpr auto  kVisionTimeout = std::chrono::milliseconds(600);

    // 识别停点: 剩 0.3m 处停 3 秒识别足球/可乐/橙球
    static constexpr float kScanStopMargin = 0.3f;   // 距终点还剩多少米时停
    static constexpr int   kScanHoldFrames = 300;    // 停 3 秒 @100Hz
    static constexpr float kPostScanFwd   = 0.7f;    // 识别完固定前进 0.7m

    // 跌倒与恢复
    static constexpr float kFallRollThresh  = 0.45f;
    static constexpr float kFallPitchThresh = 0.60f;
    static constexpr float kRecoverRollOk   = 0.20f;
    static constexpr float kRecoverPitchOk  = 0.25f;
    static constexpr int   kMaxRecoverAttempts = 2;
    static constexpr auto  kRecoveryStabilize = std::chrono::milliseconds(4000);
    static constexpr auto  kGaitReengage     = std::chrono::milliseconds(1500);

    enum class State {
        WAIT_FOR_SENSORS,
        FWD_1M,         // 前进1m（entry 朝向）
        TURN_R_IN,      // 右转90°进通道
        LANE_OUT,       // 去程2.8m：限高杆低姿 + 可乐/足球撞击
        PASS_LIMBAR,    // 低姿前进1m（energy_saving 0.13）
        RISE,           // 起立（energy=false 恢复身高参数后回 LANE_OUT）
        KNOCK,          // 对齐+撞击可乐/足球
        SCAN_STOP,      // 剩 0.3m 处停 5 秒识别播报
        TURN_BACK,      // 掉头180°
        LANE_BACK,      // 回程2.8m：只处理限高杆
        TURN_R_OUT,     // 右转90°回 entry 朝向（进下一轮）
        TURN_SPEC_L,    // test 路线 第3轮绕行: 左转90°
        FWD_SPEC_1,     // 前进1.0m
        TURN_SPEC_R,    // 右转90°
        FWD_SPEC_2,     // 前进1.1m
        TURN_SPEC_L2,   // 左转90°进通道
        FWD_LINK_1,     // test 路线2: 第1轮后前进1.8m
        FWD_LINK_2,     // 第2轮左转后前进2.5m
        TURN_LINK_L,    // 最后左转90°立正
        TURN_L_FINAL,   // 第3轮回程后左转90°离场
        FWD_EXIT_1,     // 不规则四边形离场: 前进1.0m
        TURN_EXIT_R,    // 右转90°
        FWD_EXIT_2,     // 前进1.0m
        TURN_EXIT_L,    // 左转90°
        FWD_EXIT_3,     // 前进1.5m
        TURN_EXIT_L2,   // 前进1.5m后再左转90°
        RECOVERING,
        DONE
    } state_{State::WAIT_FOR_SENSORS};

    bool done_{false};
    bool odom_initialized_{false};

    // 朝向（转向源 abs_yaw，与 Stage1/2 同约定）
    float entry_yaw_{0.0f};   // 每轮起点朝向
    float lane_yaw_{0.0f};    // 通道内朝向 = entry_yaw - 90°
    float back_yaw_{0.0f};    // 返回朝向 = entry_yaw + 90°

    // 轮次
    int  round_count_{0};     // 已完成轮数 0..kMaxRounds

    // 恢复
    bool recovery_cmd_sent_{false};
    bool gait_reengaged_{false};
    int  recovery_attempts_{0};
    State recovery_return_{State::FWD_1M};   // 恢复完成后回哪个状态

    // 帧计数/视觉序列
    int  ready_frames_{0};
    int  state_frames_{0};
    int  sub_state_{0};
    std::uint64_t last_vision_seq_{0};

    // 里程
    float ref_x_{0.0f};
    float ref_y_{0.0f};
    float travelled_since_ref_{0.0f};
    float last_odom_x_{0.0f};
    float last_odom_y_{0.0f};

    // 转向位置保持 (原地转向漂移, 记录起点拉回)
    float turn_ref_x_{0.0f};
    float turn_ref_y_{0.0f};
    bool  turn_ref_valid_{false};

    // 相对转向: 进入时快照当前 yaw+增量, 物理转满固定角度
    bool  turn_rel_valid_{false};
    float turn_rel_target_{0.0f};
    float exit_yaw1_{0.0f};   // 离场: 左转90°后方向
    float exit_yaw2_{0.0f};   // 离场: 右转90°后方向
    float exit_yaw3_{0.0f};   // 离场: 左转90°后方向
    float spec_yaw1_{0.0f};   // test 绕行: 左转90°后方向
    float spec_yaw2_{0.0f};   // test 绕行: 右转90°后方向
    float link_yaw1_{0.0f};   // test 路线2: 第1轮后方向
    float link_yaw2_{0.0f};   // test 路线2: 第2轮左转后方向

    // 子任务临时量
    float sub_start_travel_{0.0f};   // 低姿/撞击段起点里程（相对 travelled_since_ref_）
    float knock_cx_{0.0f};           // 撞击目标横向偏移
    float target_yaw_{0.0f};
    State rise_return_{State::LANE_OUT};  // 起立后回哪个状态（去程/回程）

    bool  pitch_unlocked_{false};   // x_effect_scale_pos 是否已破限

    // 语音播报去重（每轮复位）
    bool  tts_limbar_{false};
    bool  tts_coke_{false};
    bool  tts_football_{false};
    bool  tts_ball_{false};
    bool  tts_obstacle_{false};   // 蓝色方块播报
    bool  post_scan_{false};      // 识别停点结束后固定前进 0.5m 标志

    // 识别停点
    State scan_return_{State::LANE_OUT};  // 停点结束后回哪
    bool  scan_done_out_{false};          // 去程已停过 (回程不需要停)
    int   scan_frames_{0};

    std::chrono::steady_clock::time_point last_vision_time_{};
    std::chrono::steady_clock::time_point recovery_start_{};

    State last_log_state_{State::WAIT_FOR_SENSORS};  // 状态切换日志去重

    static const char* state_name(State s);

    void update_odometry();
    void update_perception();
    void update_perception_in_state();

    bool walk_distance(float distance, float target_yaw, float speed);
    // 低姿行走：7671 value&0x02（身高强制0.13），带 yaw 闭环，走满 distance
    bool walk_low(float distance, float target_yaw, float speed);
    bool turn_to_yaw(float target_yaw, float rel_delta = 0.0f);   // rel_delta≠0: 相对转向, 从当前yaw转固定角度
    bool align_to_target(float cx, float tol = 0.12f);

    void lane_out();
    void pass_limbar();
    void rise();
    void knock();
    void lane_back();
    void scan_stop();
    void lane_pitch_unlock();   // 2.8m 段全程低头: 进段破限
    void lane_pitch_restore();  // 出段恢复限位

    void enter_recovery();
    void handle_recovering();
    void finish();

    static float clamp(float v, float lo, float hi);
};
