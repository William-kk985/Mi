#pragma once

#include <chrono>
#include <cstdint>

#include "cyberdog_race/stages/stage_base.hpp"

// Stage 4 real-track: 深隧寻珍
//
// 衔接与通道流程：
//   1. stage3结束 → 左转90° → 里程计走2.7m → 右转90° → 进入通道1(最左)
//   2. 通道内边前进边识别：
//      - 限高杆：降低身高~35cm（杆高40cm）→ 前进50cm → 身高恢复 → 继续
//      - 可乐/橙球/足球：完成任务 → 转180° → 原路返回通道入口
//   3. 通道1回到入口 → 左转90°→走1m→左转90° → 进入通道2（左→中→右）
//   4. 通道内遇蓝色障碍：原地踏步+左右扫线（左90→右90→右90→左90）
//      → 扫到虚线即跨过去绕行 → 继续通道任务
//   5. 全部3通道完成 → 找独木桥 → 前腿足底碰到 → DONE
class Stage4Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override;

    float get_desired_height()     const override { return crouch_active_ ? kCrouchHeight : kStandHeight; }
    float get_desired_step_height() const override { return crouch_active_ ? kCrouchStepH   : kStandStepH; }

private:
    // 身高参数（限高杆高40cm → 蹲到35cm留裕度）
    static constexpr float kStandHeight  = 0.24f;   // 站立身高
    static constexpr float kCrouchHeight = 0.35f;   // 限高杆下通过身高（~35cm，杆40cm）
    static constexpr float kStandStepH   = 0.10f;
    static constexpr float kCrouchStepH  = 0.06f;   // 限高杆下步高稍大防蹭

    // 运动参数
    static constexpr float kWalkSpeed    = 0.30f;
    static constexpr float kCrawlSpeed   = 0.15f;
    static constexpr float kBackSpeed    = 0.18f;
    static constexpr float kKnockSpeed   = 0.25f;
    static constexpr float kTurnRate     = 0.35f;
    static constexpr float kYawKp        = 0.8f;
    static constexpr float kYawTol       = 0.08f;

    // 任务参数
    static constexpr float kCokeKnock        = 1.20f;
    static constexpr float kOrangePokeDist   = 0.30f;
    static constexpr float kOrangeBackDist   = 0.20f;
    static constexpr float kFootballKickDist = 1.50f;
    static constexpr float kLimbarWalkDist   = 0.50f;   // 限高杆下蹲走50cm就恢复
    static constexpr float kClearAfterHandle = 0.50f;
    static constexpr float kNavigateSpeed    = 0.15f;
    static constexpr float kChannelEnterDist = 0.80f;   // 进通道走到限高杆下
    static constexpr float kSwitchWalkDist   = 1.0f;    // 通道间切换走1m
    static constexpr float kMaxSeekBridgeDist= 3.0f;
    static constexpr float kBridgeClearance   = 0.15f;
    static constexpr float kBridgeFrontDist   = 0.20f;

    // 蓝障碍绕行参数（横移方案）
    static constexpr float kObsShift      = 0.50f;   // 第1步：左右横移距离（跨过虚线到邻道）
    static constexpr float kObsFwdStep    = 0.20f;   // 第2步：直走距离（绕过障碍正前方）
    static constexpr float kObsDiagDist   = 0.55f;   // 第3步：45°斜向回赛道距离（横移回0.5m同时前进）
    static constexpr float kObsShiftSpeed = 0.15f;   // 横移速度 m/s
    static constexpr float kObsDiagSpeed  = 0.20f;   // 斜向行走速度 m/s

    // 3-4衔接与通道数量
    static constexpr float kInitWalkDist    = 2.7f;     // 左转后走2.7m到通道1
    static constexpr int   kMaxChannels     = 3;
    static constexpr float kDividerStopDist = 0.30f;    // 黄实线<0.3m停（通道尽头边界）

    // 阈值与超时
    static constexpr int   kReadyFrames    = 5;
    static constexpr int   kMaxLostFrames  = 4;
    static constexpr int   kCrouchFrames   = 40;
    static constexpr int   kDetectMaxFrames= 300;
    static constexpr auto  kVisionTimeout  = std::chrono::milliseconds(600);

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
        INIT_ALIGN,         // 3-4衔接：左转→2.7m→右转→进入通道1

        NAVIGATE_CH,        // 通道内：前进扫描
        PASS_LIMBAR,        // 通道内：限高杆（蹲35cm→走50cm→起）
        HANDLE_COKE,
        HANDLE_ORANGE_BALL,
        HANDLE_FOOTBALL,
        OBSTACLE_SCAN,      // 蓝障碍：原地踏步+左右扫线找虚线
        OBSTACLE_CROSS,     // 蓝障碍：跨虚线绕行
        RETURN_ENTRANCE,    // 任务完成：转180°→原路返回通道入口

        SWITCH_CHANNEL,     // 通道间切换：左转90°→走1m→左转90°→进下个通道

        SEEK_BRIDGE,
        RECOVERING,
        DONE
    } state_{State::WAIT_FOR_SENSORS};

    bool done_{false};
    bool odom_initialized_{false};
    bool crouch_active_{false};

    // 朝向
    float entry_yaw_{0.0f};     // stage3结束朝向（上行）
    float channel_yaw_{0.0f};   // 通道内朝向 = entry_yaw_
    float switch_yaw_{0.0f};    // 通道切换朝向 = entry_yaw_ + π/2（向右走）

    // 通道
    int  channel_count_{0};     // 已处理通道数 0..2
    bool in_channel_{false};
    float entrance_x_{0.0f};    // 通道入口坐标（返回用）
    float entrance_y_{0.0f};

    // 限高杆在通道内返回路径上
    int   limbars_on_path_{0};  // 本条通道已经过的限高杆数量（返回时要再过一遍）
    float last_limbar_dist_{0.0f};

    // 障碍绕行（横移方案）
    bool  dashed_found_{false};
    float dashed_yaw_{0.0f};
    int   obstacle_side_{0};     // +1=虚线在右→右平移；-1=虚线在左→左平移（由 divider_x 符号决定）

    // 恢复
    bool recovery_cmd_sent_{false};
    bool gait_reengaged_{false};
    int  recovery_attempts_{0};

    // 帧计数/视觉序列
    int  ready_frames_{0};
    int  detect_wait_{0};
    int  lost_frames_{0};
    int  state_frames_{0};
    int  sub_state_{0};
    std::uint64_t last_vision_seq_{0};

    // 里程
    float ref_x_{0.0f};
    float ref_y_{0.0f};
    float travelled_since_ref_{0.0f};
    float total_travelled_{0.0f};
    float last_odom_x_{0.0f};
    float last_odom_y_{0.0f};

    // 临时
    float target_yaw_{0.0f};
    bool  clearing_after_handle_{false};
    int   processed_count_{0};

    // 语音播报去重（每个目标每通道只播报一次；进新通道/新目标时复位）
    bool  tts_limbar_{false};
    bool  tts_obstacle_{false};
    bool  tts_coke_{false};
    bool  tts_ball_{false};
    bool  tts_football_{false};

    std::chrono::steady_clock::time_point last_vision_time_{};
    std::chrono::steady_clock::time_point recovery_start_{};

    void update_odometry();
    void update_perception();
    void update_perception_in_state();

    bool walk_distance(float distance, float target_yaw, float speed);
    // 横移/斜向行走：x_dir/y_dir 为单位方向向量（归一化），保持 target_yaw 朝向
    //   例：纯右横移 x_dir=0 y_dir=1；左前45° x_dir=0.707 y_dir=-0.707
    bool walk_xy_distance(float distance, float x_dir, float y_dir,
                          float target_yaw, float speed);
    bool turn_to_yaw(float target_yaw);
    bool align_to_target(float cx, float tol = 0.12f);

    void init_align();
    void switch_channel();
    void pass_limbar();
    void handle_coke();
    void handle_orange_ball();
    void handle_football();
    void obstacle_scan();
    void obstacle_cross();
    void return_entrance();
    void seek_bridge();

    void finish_handle(bool need_return);

    void enter_recovery();
    void handle_recovering();
    void finish();

    static float clamp(float v, float lo, float hi);
};
