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

constexpr float WALK_V        = 0.30f;   // 前进速度 m/s
constexpr float GOAL_Y        = 3.6f;    // 左转90°方向前进 3.6m (2026-08-12)
constexpr float STEP_H        = 0.17f;   // 步高 (与 Stage1 一致)

}  // namespace

void Stage2Real::init() {
    phase_ = Phase::NAV;
    done_  = false;
    // 目标点: 起点左转 90° 方向、3.6m 处
    // (goto_relative 自动: 先转向对准 +y 方向=左转90°, 再走 3.6m)
    goto_relative(0.0f, GOAL_Y, WALK_V, STEP_H);
    RCLCPP_INFO(rclcpp::get_logger("stage2_real"),
                "[Stage2Real] init: 目标点(左转90°方向 %.1fm)", GOAL_Y);
}

void Stage2Real::run() {
    if (done_) return;
    if (phase_ == Phase::NAV) {
        if (update_nav()) {          // 自动: 转向对准右转5°方向 → 走 1m
            phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 目标点到达, DONE\n");
            fflush(stderr);
#endif
        }
        return;
    }
    if (phase_ == Phase::DONE) { done_ = true; }
}
