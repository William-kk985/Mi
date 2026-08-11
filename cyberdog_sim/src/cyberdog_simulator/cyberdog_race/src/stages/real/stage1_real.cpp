#include "cyberdog_race/stages/real/stage1_real.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage1Real 真机版 — 石径探路
// 步高 0.15 | 前进 6m | 视觉巡线 + 里程计 + IMU 转90°
// 真机接口: set_walk_velocity_step = 303 WALK_USERTROT + step_height
//   (motion_servo_cmd.step_height 字段, 真机正确步高通道 2026-08-08 确认)
// ═══════════════════════════════════════════════════════════

namespace {

constexpr float STEP_H        = 0.15f;   // 步高 0.15 (过 5cm 石板足够)
constexpr float WALK_V        = 0.30f;   // 正常前进速度 m/s
constexpr float RUSH_V        = 0.50f;   // 卡住冲刺速度
constexpr float GOAL_DIST     = 6.0f;    // 前进 6m (里程计判定)
constexpr float TURN_YAW      = M_PI / 2.0f;  // 目标转角 90°
constexpr float KP_YAW        = 0.8f;    // 视觉比例
constexpr float KD_YAW        = 0.3f;    // 视觉微分
constexpr float IMU_WEIGHT    = 0.3f;    // IMU 回正权重(视觉为主)
constexpr float STUCK_DIST    = 0.01f;   // 卡住判定位移阈值 (m)
constexpr int   STUCK_THRESH  = 30;      // 卡住帧数
constexpr int   RUSH_FRAMES   = 20;      // 冲刺帧数
constexpr int   LANE_LOST_LIM = 10;      // 丢线容忍帧数

}  // namespace

void Stage1Real::init() {
    phase_       = Phase::FORWARD;
    done_        = false;
    stuck_       = 0;
    rush_        = 0;
    lane_lost_   = 0;
    prev_offset_ = 0.0f;
    start_x_     = sensor_.odom_x;
    start_y_     = sensor_.odom_y;
    start_yaw_   = sensor_.yaw;
    last_x_      = sensor_.odom_x;
    last_y_      = sensor_.odom_y;

    // ── 站起: 真机必须走 MotionResultCmd 111 (RECOVERYSTAND) ──
    // ⚠ locomotion() 是 gamepad 接口, 真机无效 (2026-08-11 实测狗不站)
    //   站起是异步服务 → 先等服务就绪, 再等待真正站起
    motion_.wait_motion_result_ready(5);
    motion_.stand();                                         // RECOVERYSTAND 官方站立
    rclcpp::sleep_for(std::chrono::seconds(3));              // 等真正站起(服务异步)

    motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H); // 预热: 303+步高0.15
    motion_.set_body_pitch(-0.10f);                          // 微微抬头(真机负值=抬头), 防低头撞石板
    RCLCPP_INFO(rclcpp::get_logger("stage1_real"), "[Stage1Real] init: 步高%.2f 前进%.1fm", STEP_H, GOAL_DIST);
}

void Stage1Real::run() {
    if (done_) return;

    // ── ② 原地转 90° ─────────────────────────────────────
    if (phase_ == Phase::TURN) {
        float yaw_err = norm_yaw(start_yaw_ + TURN_YAW - sensor_.yaw);
        if (std::abs(yaw_err) < 0.05f) {                     // 转到位
            motion_.stop();
            phase_ = Phase::DONE;
        } else {
            float turn = std::max(0.10f, std::min(0.45f, std::abs(yaw_err) * 0.6f));
            motion_.set_walk_velocity_step(0.0f, 0.0f, yaw_err > 0 ? turn : -turn, STEP_H);
        }
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; return; }

    // ── ① 前进 6m 巡线 ───────────────────────────────────
    float dist = std::hypot(sensor_.odom_x - start_x_, sensor_.odom_y - start_y_);
    if (dist >= GOAL_DIST) {                                 // 走完 6m
        motion_.stop();
        phase_ = Phase::TURN;
        return;
    }

    // 卡住检测: 位移过小连续 N 帧 → 冲刺脱困(加速冲过石板)
    float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;

    if (rush_ > 0) {                                         // 冲刺中
        motion_.set_walk_velocity_step(RUSH_V, 0.0f, 0.0f, STEP_H);
        --rush_;
        return;
    }

    if (moved < STUCK_DIST) {
        if (++stuck_ >= STUCK_THRESH) {
            stuck_ = 0;
            rush_ = RUSH_FRAMES;
            RCLCPP_WARN(rclcpp::get_logger("stage1_real"), "[Stage1Real] 卡住! 冲刺脱困");
        }
    } else {
        stuck_ = 0;
    }

    // 丢线保护: 视觉失效超限 → 纯 IMU 直行(靠 yaw 保持方向)
    if (!sensor_.lane_valid) {
        if (++lane_lost_ > LANE_LOST_LIM) {
            motion_.set_walk_velocity_step(WALK_V, 0.0f, -sensor_.yaw * 0.8f, STEP_H);
        }
        return;
    }
    lane_lost_ = 0;

    // 巡线: 视觉 PD + IMU 回正(视觉为主)
    float d_offset = sensor_.lane_offset - prev_offset_;
    prev_offset_   = sensor_.lane_offset;

    float vis_cmd = -(KP_YAW * sensor_.lane_offset + KD_YAW * d_offset);
    float imu_cmd = norm_yaw(-sensor_.yaw) * 0.3f;
    float yaw_cmd = imu_cmd * IMU_WEIGHT + vis_cmd * (1.0f - IMU_WEIGHT);
    yaw_cmd = std::max(-0.5f, std::min(0.5f, yaw_cmd));

    motion_.set_walk_velocity_step(WALK_V, 0.0f, yaw_cmd, STEP_H);
}
