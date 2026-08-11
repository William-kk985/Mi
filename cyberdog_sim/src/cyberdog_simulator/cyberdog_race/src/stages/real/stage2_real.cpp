#include "cyberdog_race/stages/real/stage2_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage2Real 真机版 — 第2赛段
// 先右转 5°(修正 Stage1 转向偏差) → 前进 1.5m
// 方向反馈用 abs_yaw(global_to_robot.rpy[2]), IMU yaw 真机恒0 别用 (README 2026-08-11)
// ═══════════════════════════════════════════════════════════

namespace {

constexpr float CORRECT_YAW   = 5.0f * M_PI / 180.0f;  // 右转 5° 修正 (Stage1 转向偏差)
constexpr float WALK_V        = 0.30f;   // 前进速度 m/s
constexpr float GOAL_DIST     = 1.5f;    // 前进 1.5m
constexpr float STEP_H        = 0.17f;   // 步高 (与 Stage1 一致)
constexpr float KP_YAW        = 0.8f;    // 回正增益

}  // namespace

void Stage2Real::init() {
    phase_         = Phase::CORRECT;
    done_          = false;
    correct_guard_ = 0;
    start_yaw_     = sensor_.abs_yaw;   // 记录当前朝向(Stage1 转完 90° 后的朝向)
    start_x_       = sensor_.odom_x;
    start_y_       = sensor_.odom_y;
    last_x_        = sensor_.odom_x;
    last_y_        = sensor_.odom_y;
    traveled_      = 0.0f;
    RCLCPP_INFO(rclcpp::get_logger("stage2_real"), "[Stage2Real] init: 右转5°修正 前进%.1fm", GOAL_DIST);
}

void Stage2Real::run() {
    if (done_) return;

    // ── ① 先右转 5° 修正 (Stage1 转向偏差, abs_yaw 闭环) ──
    if (phase_ == Phase::CORRECT) {
        float yaw_err = norm_yaw(start_yaw_ - CORRECT_YAW - sensor_.abs_yaw);  // 右转 = yaw 减小
        bool ok = (correct_guard_ > 10) && (std::abs(yaw_err) < 0.02f);  // 至少0.1s + 容差2°
        correct_guard_++;
        if (ok) {
            phase_ = Phase::FORWARD;
            // 转完后更新前进基准
            start_yaw_ = sensor_.abs_yaw;
            start_x_   = sensor_.odom_x;
            start_y_   = sensor_.odom_y;
            last_x_    = sensor_.odom_x;
            last_y_    = sensor_.odom_y;
            traveled_  = 0.0f;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 右转5°完成, 前进%.1fm\n", GOAL_DIST);
            fflush(stderr);
#endif
        } else {
            float turn = std::max(-0.15f, std::min(0.15f, yaw_err * 0.8f));  // 小角度慢速
            motion_.set_walk_velocity_step(0.0f, 0.0f, turn, STEP_H);
#ifdef DEBUG_STAGE
            if (correct_guard_ % 20 == 0) {
                fprintf(stderr, "[S2Stage] 右转修正中 err=%.2f absYaw=%.2f\n",
                        yaw_err, sensor_.abs_yaw);
                fflush(stderr);
            }
#endif
        }
        return;
    }

    // ── ② 前进 1.5m (累计位移) ──
    if (phase_ == Phase::FORWARD) {
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x;
        last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;   // 定位跳变保护
        traveled_ += moved;

        if (traveled_ >= GOAL_DIST) {
            motion_.stop();
            phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 前进 %.2fm 完成, DONE\n", traveled_);
            fflush(stderr);
#endif
            return;
        }

        // 回正用相对转完后的朝向
        float yaw_cmd = std::max(-0.3f, std::min(0.3f, -(sensor_.abs_yaw - start_yaw_) * KP_YAW));
#ifdef DEBUG_MOTION
        static int dbg_m_ = 0;
        if (++dbg_m_ % 10 == 0) {
            fprintf(stderr, "[S2M] cmd v=(%.2f,0,%.2f) 步高%.2f\n", WALK_V, yaw_cmd, STEP_H);
            fflush(stderr);
        }
#endif
        motion_.set_walk_velocity_step(WALK_V, 0.0f, yaw_cmd, STEP_H);
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; }
}
