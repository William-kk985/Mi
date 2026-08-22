#pragma once

#include <chrono>
#include <cstdint>

#include "cyberdog_race/debug_config.hpp"
#include "cyberdog_race/stages/stage_base.hpp"

// Stage 4 real-track: 蛇形三通道巡逻（2026-08-16 新版动作）
//
// 每轮动作：
//   前进1m → 右转90° → 前进2.8m（去程检测）→ 掉头180° → 回来2.8m（回程检测）→ 右转90°
// 共3轮（蛇形扫3条平行通道，全右转衔接）
//
// 去程2.8m内检测：
//   - 限高杆 <1.2m → 低姿前进1m（官方 use_energy_saving_mode=0 → 身高强制0.13）→ 起立继续
//   - 可乐/足球 ≤0.8m → 冲过去撞击 → 继续走到2.8m终点
// 回程2.8m内检测：只处理限高杆（低姿1m），不撞可乐
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

private:
    // 身高/步高
    static constexpr float kStandHeight = 0.24f;
    static constexpr float kStandStepH  = 0.10f;

    // 路径参数
    static constexpr int   kMaxRounds = 3;         // 三轮
#if defined(DEBUG_STAGE4_TEST_ROUTE) && defined(DEBUG_STAGE4_TEST_ROUTE2)
#error "DEBUG_STAGE4_TEST_ROUTE 与 DEBUG_STAGE4_TEST_ROUTE2 互斥, 只能开一个"
#elif defined(DEBUG_STAGE4_TEST_ROUTE)
    // (2026-08-22 test路线: 轮1起步1.8 轮2起步1.1 轮3起步1.2; 轮2去程3.25 轮3去程2.3 轮1/2/3回程3.6/3.5/3.6)
    static constexpr float kFwdDist[kMaxRounds] = {1.8f, 1.1f, 1.2f};
    static constexpr float kLaneDist[kMaxRounds] = {3.5f, 3.25f, 2.3f};
    static constexpr float kLaneDistBack[kMaxRounds] = {3.6f, 3.5f, 3.6f};
#elif defined(DEBUG_STAGE4_TEST_ROUTE2)
    // (2026-08-22 test路线2/第三版: 轮1起步0.8 轮2起步1.2, 通道3.5/3.6 与 3.35/3.6, 轮1后前1.8, 轮2后左转→前2.5→左转立正)
    static constexpr float kFwdDist[kMaxRounds] = {0.8f, 1.2f, 1.0f};
    static constexpr float kLaneDist[kMaxRounds] = {3.5f, 3.35f, 2.0f};
    static constexpr float kLaneDistBack[kMaxRounds] = {3.6f, 3.6f, 2.1f};
#else
    // (2026-08-18 用户: 三轮起步前进 0.8m/1m/1m; 先第1轮0.8m, 第2轮1m)
    // (2026-08-22 用户: 轮2 1.0→1.2m)
    static constexpr float kFwdDist[kMaxRounds] = {0.8f, 1.1f, 1.0f};   // (2026-08-22 用户: 轮2=1.1)
    static constexpr float kLaneDist[kMaxRounds] = {3.5f, 3.35f, 2.0f};  // 去程按轮次 (2026-08-22 用户: 轮2=3.35 轮3=2.0(3.0-1.0起点后移))
    static constexpr float kLaneDistBack[kMaxRounds] = {3.55f, 2.45f, 2.25f}; // 回程按轮次 (2026-08-22 用户: 回程不够, 全部+0.15)
#endif
    // (2026-08-22 test路线第3轮绕行, 仅test进入; 始终定义避免非test编译报未定义)
    static constexpr float kSpecialFwd1 = 1.1f;  // 绕行: 左转90°后前进 (2026-08-22 用户: 1.0→1.1)
    static constexpr float kSpecialFwd2 = 0.7f;  // 绕行: 右转90°后前进 (2026-08-22 用户: 0.6→0.7)
    // (2026-08-22 test路线2/第三版链接段, 始终定义)
    static constexpr float kLinkFwd1 = 1.8f;   // 第1轮后前进1.8m
    static constexpr float kLinkFwd2 = 2.5f;   // 第2轮左转后前进2.5m

    // 运动参数
    static constexpr float kWalkSpeed = 0.30f;
    static constexpr float kLowSpeed  = 0.20f;     // 限高杆下低姿速度
    static constexpr float kKnockSpeed= 0.35f;     // 撞击可乐/足球速度
    static constexpr float kTurnRate  = 0.35f;
    static constexpr float kYawKp     = 0.8f;
    static constexpr float kYawTol    = 0.03f;   // (2026-08-21 0.08→0.03: 旧值4.6°太宽致转向时多时少, 对齐Stage2收紧)
    // (2026-08-21 归零: “物理欠转2°”旧假设在Stage2已证不成立, 转满; 与Stage2对齐)
    static constexpr float kTurnExtraRad = 0.0f;
    static constexpr float kTurnYawBias  = 0.05f;   // (2026-08-22 恢复git蓝障版那套: 0.08→0.05, 正式/test2统一)
    static constexpr float kYawDeadband = 0.01f;  // (2026-08-17 行进yaw死区, 防微差抖动, Stage2同款)
#ifdef DEBUG_STAGE4_NO_COMP
    // (2026-08-21 startrace4test: 左补全0, 观察自然偏左量; 正式版保持注释走#else)
    static constexpr float kFwdLatComp  = 0.0f;
    static constexpr float kFwdLatCompLow = 0.0f;
#else
    static constexpr float kFwdLatComp  = 0.020f; // (2026-08-22 用户: test和正式都偏左, 0.018→0.020 右补加大)
    static constexpr float kFwdLatCompLow = 0.017f; // (2026-08-22 同步 0.015→0.017)
#endif
    static constexpr float kTurnLatComp = -0.01f;   // (2026-08-22 原地转向右漂: 固定向左补偿, 负=左)
    static constexpr float kTurnHoldLimit = 0.12f;  // (2026-08-22 转向位置保持限幅 0.08→0.12)

    // 任务参数
    static constexpr float kLimbarTriggerDist = 0.8f;  // 限高杆<0.8m触发低姿 (2026-08-17 用户: 1.2→0.8)
    static constexpr float kLimbarPitch       = 0.25f; // 限高杆下低头~14° (2026-08-17 用户: 0.20→0.25 大一些)
    // 低头破限 (2026-08-17): 303走路pitch硬限±5.7°大命令无效(test17),
    //   x_effect_scale_pos=+30放大限位→14°保持走满(test19✅); 过杆后复原默认-0.55
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

    // 识别停点 (2026-08-17 用户: 剩0.3m处停8秒识别足球/可乐/橙球)
    static constexpr float kScanStopMargin = 0.3f;   // 距终点还剩多少米时停 (2026-08-21 用户: 0.6→0.3)
    static constexpr int   kScanHoldFrames = 300;    // 停3秒 @100Hz (2026-08-22 用户: 8秒→3秒)
    static constexpr float kPostScanFwd   = 0.7f;    // (2026-08-22 用户: 识别完固定前进 0.5→0.7m)
#ifdef DEBUG_STAGE4_TEST_ROUTE
    static constexpr float kExitFwd1 = 0.8f;  // (2026-08-22 test路线: 左转90°后前进0.8m, 再左转90°立正)
    static constexpr float kExitFwd2 = 1.15f; // test不用
    static constexpr float kExitFwd3 = 1.5f;  // test不用
#else
    static constexpr float kExitFwd1 = 1.0f;  // (2026-08-22 不规则四边形离场: 左转90°后前进1.0)
    static constexpr float kExitFwd2 = 1.15f; // 右转90°后前进 (2026-08-22 用户: 1.0→1.15, 绕第三赛道蓝色障碍物)
    static constexpr float kExitFwd3 = 1.5f;  // 左转90°后前进1.5
#endif

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
        SCAN_STOP,      // (2026-08-17) 剩0.3m处停5秒识别播报
        TURN_BACK,      // 掉头180°
        LANE_BACK,      // 回程2.8m：只处理限高杆
        TURN_R_OUT,     // 右转90°回 entry 朝向（进下一轮）
        TURN_SPEC_L,    // (2026-08-22 test路线 第3轮绕行: 左转90°)
        FWD_SPEC_1,     // 前进1.0m
        TURN_SPEC_R,    // 右转90°
        FWD_SPEC_2,     // 前进1.1m
        TURN_SPEC_L2,   // 左转90°进通道
        FWD_LINK_1,     // (2026-08-22 test路线2: 第1轮后前进1.8m)
        FWD_LINK_2,     // 第2轮左转后前进2.5m
        TURN_LINK_L,    // 最后左转90°立正
        TURN_L_FINAL,   // (2026-08-20) 第3轮回程后左转90°离场
        FWD_EXIT_1,     // (2026-08-22 不规则四边形离场: 前进1.0m)
        TURN_EXIT_R,    // 右转90°
        FWD_EXIT_2,     // 前进1.0m
        TURN_EXIT_L,    // 左转90°
        FWD_EXIT_3,     // 前进1.5m
        TURN_EXIT_L2,   // (2026-08-22 用户: 前进1.5m后再左转90°)
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

    // 转向位置保持 (2026-08-22 用户: 原地转向漂移, 记录起点拉回)
    float turn_ref_x_{0.0f};
    float turn_ref_y_{0.0f};
    bool  turn_ref_valid_{false};

    // 相对转向 (2026-08-22 用户: 转向转少 → 进入时快照当前yaw+增量, 物理转满固定角度)
    bool  turn_rel_valid_{false};
    float turn_rel_target_{0.0f};
    float exit_yaw1_{0.0f};   // 离场: 左转90°后方向
    float exit_yaw2_{0.0f};   // 离场: 右转90°后方向
    float exit_yaw3_{0.0f};   // 离场: 左转90°后方向
    float spec_yaw1_{0.0f};   // (2026-08-22 test绕行: 左转90°后方向)
    float spec_yaw2_{0.0f};   // (2026-08-22 test绕行: 右转90°后方向)
    float link_yaw1_{0.0f};   // (2026-08-22 test路线2: 第1轮后方向)
    float link_yaw2_{0.0f};   // (2026-08-22 test路线2: 第2轮左转后方向)

    // 子任务临时量
    float sub_start_travel_{0.0f};   // 低姿/撞击段起点里程（相对 travelled_since_ref_）
    float knock_cx_{0.0f};           // 撞击目标横向偏移
    float target_yaw_{0.0f};
    State rise_return_{State::LANE_OUT};  // 起立后回哪个状态（去程/回程）

    bool  pitch_unlocked_{false};   // x_effect_scale_pos 是否已破限 (2026-08-17)

    // 语音播报去重（每轮复位）
    bool  tts_limbar_{false};
    bool  tts_coke_{false};
    bool  tts_football_{false};
    bool  tts_ball_{false};
    bool  tts_obstacle_{false};   // (2026-08-18 蓝色方块播报)
    bool  post_scan_{false};      // (2026-08-21 识别停点结束后固定前进0.5m标志)

    // 识别停点 (2026-08-17)
    State scan_return_{State::LANE_OUT};  // 停点结束后回哪
    bool  scan_done_out_{false};          // 去程已停过 (2026-08-18 回程停点已删, 用户: 回来不需要)
    int   scan_frames_{0};

    std::chrono::steady_clock::time_point last_vision_time_{};
    std::chrono::steady_clock::time_point recovery_start_{};

    State last_log_state_{State::WAIT_FOR_SENSORS};  // (2026-08-17 状态切换日志去重)

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
    void lane_pitch_unlock();   // (2026-08-18) 2.8m段全程低头: 进段破限
    void lane_pitch_restore();  // 出段恢复限位

    void enter_recovery();
    void handle_recovering();
    void finish();

    static float clamp(float v, float lo, float hi);
};
