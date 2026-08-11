#include "cyberdog_race/stages/real/stage2_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage2Real 真机版 — 第2赛段
// 两个相对点位 (goto_relative, 不借助地图):
//   点位1: 右转 5° 走 1.2m   → goto_relative(1.195, -0.105)
//   点位2: 左转 85° 走 3.4m  → goto_relative(0.296, 3.387) (相对到达点1时的朝向)
// 方向反馈用 abs_yaw(global_to_robot.rpy[2]), IMU yaw 真机恒0 别用 (README 2026-08-11)
// ═══════════════════════════════════════════════════════════

namespace {

constexpr float WALK_V = 0.30f;   // 前进速度 m/s
constexpr float STEP_H = 0.17f;   // 步高 (与 Stage1 一致)

// 点位1: 右转 5° 走 1.0m (cos5°=0.99619, sin5°=0.08716; 2026-08-12 1.2m→1m)
constexpr float NAV1_X = 1.0f * 0.99619f;   // +0.996 (右转=负 y)
constexpr float NAV1_Y = -1.0f * 0.08716f;  // -0.087
// 点位2: 左转 90° 走 3.4m (cos90°=0, sin90°=1; 2026-08-12 85°→90°), 相对当前朝向
constexpr float NAV2_X = 0.0f;              // 0
constexpr float NAV2_Y = 3.4f;              // 3.4

}  // namespace

void Stage2Real::init() {
    phase_ = Phase::NAV1;
    done_  = false;
    // 点位1: 自动转向右 5° → 走 1.2m
    goto_relative(NAV1_X, NAV1_Y, WALK_V, STEP_H);
    RCLCPP_INFO(rclcpp::get_logger("stage2_real"),
                "[Stage2Real] init: 点位1(右转5°,1.2m) → 点位2(左转85°,3.4m)");
}

void Stage2Real::run() {
    if (done_) return;

    if (phase_ == Phase::NAV1) {
        if (update_nav()) {          // 到达点位1
            phase_ = Phase::NAV2;
            goto_relative(NAV2_X, NAV2_Y, WALK_V, STEP_H);   // 点位2: 左转85° → 走3.4m
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 点位1到达, 转点位2(左转85°,3.4m)\n");
            fflush(stderr);
#endif
        }
        return;
    }

    if (phase_ == Phase::NAV2) {
        if (update_nav()) {          // 到达点位2
            phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 点位2到达, DONE\n");
            fflush(stderr);
#endif
        }
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; }
}
