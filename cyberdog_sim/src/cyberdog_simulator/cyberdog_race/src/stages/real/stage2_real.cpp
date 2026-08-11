#include "cyberdog_race/stages/real/stage2_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage2Real 真机版 — 第2赛段
// 前进 1.5m → 左转 90° → 再前进 3.6m
// 方向反馈用 abs_yaw(global_to_robot.rpy[2]), IMU yaw 真机恒0 别用 (README 2026-08-11)
// ═══════════════════════════════════════════════════════════

namespace {

constexpr float WALK_V      = 0.30f;   // 前进速度 m/s
constexpr float FWD1_DIST   = 1.5f;    // 第1段 前进 1.5m
constexpr float FWD2_DIST   = 3.6f;    // 第2段 前进 3.6m
constexpr float STEP_H      = 0.17f;   // 步高 (与 Stage1 一致)
constexpr float TURN_YAW    = M_PI / 2.0f;  // 左转 90°
constexpr float TURN_SPEED  = 0.60f;   // 转向速度 (test14 同款)

}  // namespace

void Stage2Real::init() {
    phase_      = Phase::FWD1;
    done_       = false;
    turn_guard_ = 0;
    start_yaw_  = sensor_.abs_yaw;      // 赛段初始朝向(左转90°基准)
    fwd1_x_     = sensor_.odom_x;
    fwd1_y_     = sensor_.odom_y;
    last_x_     = sensor_.odom_x;
    last_y_     = sensor_.odom_y;
    traveled_   = 0.0f;
    RCLCPP_INFO(rclcpp::get_logger("stage2_real"),
                "[Stage2Real] init: 前进%.1fm→左转90°→前进%.1fm", FWD1_DIST, FWD2_DIST);
}

void Stage2Real::run() {
    if (done_) return;

    // ── ② 左转 90° (test14 abs_yaw 闭环, 基于赛段初始朝向) ──
    if (phase_ == Phase::TURN) {
        float yaw_err = norm_yaw(start_yaw_ + TURN_YAW - sensor_.abs_yaw);
        bool turn_done = (turn_guard_ > 20) && (std::abs(yaw_err) < 0.05f);  // 至少0.2s防跳变
        turn_guard_++;
        if (turn_done) {
            motion_.stop();
            phase_    = Phase::FWD2;
            fwd2_x_   = sensor_.odom_x;
            fwd2_y_   = sensor_.odom_y;
            last_x_   = sensor_.odom_x;
            last_y_   = sensor_.odom_y;
            traveled_ = 0.0f;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 左转90°完成, 前进%.1fm\n", FWD2_DIST);
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(0.0f, 0.0f, TURN_SPEED, STEP_H);
#ifdef DEBUG_STAGE
            if (turn_guard_ % 20 == 0) {
                fprintf(stderr, "[S2Stage] 左转中 err=%.2f absYaw=%.2f 目标=%.2f\n",
                        yaw_err, sensor_.abs_yaw, start_yaw_ + TURN_YAW);
                fflush(stderr);
            }
#endif
        }
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; return; }

    // ── ①③ 前进 (FWD1: 1.5m / FWD2: 3.6m, 累计位移+跳变保护) ──
    float dist  = (phase_ == Phase::FWD1) ? FWD1_DIST : FWD2_DIST;
    float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;
    if (moved > 0.25f) moved = 0.0f;
    traveled_ += moved;

    if (traveled_ >= dist) {
        motion_.stop();
        if (phase_ == Phase::FWD1) {
            phase_ = Phase::TURN;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 前进%.1fm完成, 左转90°\n", FWD1_DIST);
            fflush(stderr);
#endif
        } else {
            phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 前进%.1fm完成, DONE\n", FWD2_DIST);
            fflush(stderr);
#endif
        }
        return;
    }

    // 回正基准: FWD1 用初始朝向, FWD2 用左转后朝向
    float ref_yaw = (phase_ == Phase::FWD1) ? start_yaw_ : norm_yaw(start_yaw_ + TURN_YAW);
    float yaw_cmd = std::max(-0.5f, std::min(0.5f, -(sensor_.abs_yaw - ref_yaw) * 0.8f));
#ifdef DEBUG_MOTION
    static int dbg_m_ = 0;
    if (++dbg_m_ % 10 == 0) {
        fprintf(stderr, "[S2M] cmd v=(%.2f,0,%.2f) 步高%.2f\n", WALK_V, yaw_cmd, STEP_H);
        fflush(stderr);
    }
#endif
    motion_.set_walk_velocity_step(WALK_V, 0.0f, yaw_cmd, STEP_H);
}
