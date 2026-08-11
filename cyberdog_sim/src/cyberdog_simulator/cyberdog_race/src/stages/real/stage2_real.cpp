#include "cyberdog_race/stages/real/stage2_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage2Real 真机版 — 第2赛段
// 直观动作参数: 右转 3° → 走 1.0m → 左转 90° → 走 3.2m
// 方向反馈用 abs_yaw(global_to_robot.rpy[2]), IMU yaw 真机恒0 别用 (README 2026-08-11)
// ═══════════════════════════════════════════════════════════

namespace {

// ═══ Stage2 动作参数 (直接调角度/距离, 不用算坐标) ═══
constexpr float WALK_V     = 0.30f;    // 前进速度 m/s
constexpr float STEP_H     = 0.17f;    // 步高
constexpr float TURN1_DEG  = -3.0f;    // 动作1: 右转 3° (负=右转)
constexpr float DIST1_M    = 1.0f;     // 动作2: 走 1.0m
constexpr float TURN2_DEG  = +90.0f;   // 动作3: 左转 90°
constexpr float DIST2_M    = 3.05f;    // 动作4: 走 3.05m
constexpr float TURN3_DEG  = +45.0f;   // 动作5: 左转 45° (新增 2026-08-12)
constexpr float TURN4_DEG  = -45.0f;   // 动作6: 右转 45° (新增 2026-08-12)
constexpr float TURN_SPEED = 0.60f;    // 转向速度 rad/s (test14 同款, +左转)

}  // namespace

void Stage2Real::init() {
    done_          = false;
    start_yaw_     = sensor_.abs_yaw;
    turn_base_yaw_ = sensor_.abs_yaw;   // TURN1 基准 = 当前朝向
    fwd_ref_yaw_   = sensor_.abs_yaw;
    turn_guard_    = 0;
    phase_         = Phase::TURN1;
    last_x_        = sensor_.odom_x;
    last_y_        = sensor_.odom_y;
    traveled_      = 0.0f;
    RCLCPP_INFO(rclcpp::get_logger("stage2_real"),
                "[Stage2Real] init: 右转%.0f°→走%.1fm→左转%.0f°→走%.1fm→左转%.0f°→右转%.0f°",
                TURN1_DEG, DIST1_M, TURN2_DEG, DIST2_M, TURN3_DEG, TURN4_DEG);
}

void Stage2Real::run() {
    if (done_) return;

    // ── 转向 (TURN1..TURN4): 相对"进入转向时的朝向"转 turn_deg ──
    if (phase_ == Phase::TURN1 || phase_ == Phase::TURN2 ||
        phase_ == Phase::TURN3 || phase_ == Phase::TURN4) {
        float deg = (phase_ == Phase::TURN1) ? TURN1_DEG :
                    (phase_ == Phase::TURN2) ? TURN2_DEG :
                    (phase_ == Phase::TURN3) ? TURN3_DEG : TURN4_DEG;
        float target_yaw = norm_yaw(turn_base_yaw_ + deg * M_PI / 180.0f);
        float yaw_err    = norm_yaw(target_yaw - sensor_.abs_yaw);
        bool ok = (turn_guard_ > 20) && (std::abs(yaw_err) < 0.05f);  // 至少0.2s防跳变
        turn_guard_++;
        if (ok) {
            motion_.stop();
            if (phase_ == Phase::TURN1) {        // → 走 1m
                phase_ = Phase::FWD1;
                fwd_ref_yaw_ = target_yaw;
                last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
                traveled_ = 0.0f;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 右转%.0f°完成, 走%.1fm\n", std::abs(deg), DIST1_M);
                fflush(stderr);
#endif
            } else if (phase_ == Phase::TURN2) { // → 走 3.05m
                phase_ = Phase::FWD2;
                fwd_ref_yaw_ = target_yaw;
                last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
                traveled_ = 0.0f;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 左转%.0f°完成, 走%.1fm\n", std::abs(deg), DIST2_M);
                fflush(stderr);
#endif
            } else if (phase_ == Phase::TURN3) { // → 右转 45°
                phase_ = Phase::TURN4;
                turn_guard_    = 0;
                turn_base_yaw_ = sensor_.abs_yaw;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 左转45°完成, 右转45°\n");
                fflush(stderr);
#endif
            } else {                             // TURN4 → DONE
                phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 右转45°完成, DONE\n");
                fflush(stderr);
#endif
            }
        } else {
            motion_.set_walk_velocity_step(0.0f, 0.0f,
                                           yaw_err > 0 ? TURN_SPEED : -TURN_SPEED, STEP_H);
#ifdef DEBUG_STAGE
            if (turn_guard_ % 20 == 0) {
                fprintf(stderr, "[S2Stage] 转向中 目标%.0f° err=%.2f\n", deg, yaw_err);
                fflush(stderr);
            }
#endif
        }
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; return; }

    // ── 前进 (FWD1/FWD2): 累计位移 + 跳变保护, 按 fwd_ref_yaw_ 回正 ──
    float dist  = (phase_ == Phase::FWD1) ? DIST1_M : DIST2_M;
    float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;
    if (moved > 0.25f) moved = 0.0f;
    traveled_ += moved;

    if (traveled_ >= dist) {
        motion_.stop();
        if (phase_ == Phase::FWD1) {
            phase_ = Phase::TURN2;
            turn_guard_    = 0;
            turn_base_yaw_ = sensor_.abs_yaw;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 走%.1fm完成, 左转%.0f°\n", DIST1_M, TURN2_DEG);
            fflush(stderr);
#endif
        } else {
            phase_ = Phase::TURN3;
            turn_guard_    = 0;
            turn_base_yaw_ = sensor_.abs_yaw;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 走%.1fm完成, 左转45°\n", DIST2_M);
            fflush(stderr);
#endif
        }
        return;
    }

    float yaw_cmd = std::max(-0.5f, std::min(0.5f, -(sensor_.abs_yaw - fwd_ref_yaw_) * 0.8f));
#ifdef DEBUG_MOTION
    static int dbg_m_ = 0;
    if (++dbg_m_ % 10 == 0) {
        fprintf(stderr, "[S2M] cmd v=(%.2f,0,%.2f) 步高%.2f\n", WALK_V, yaw_cmd, STEP_H);
        fflush(stderr);
    }
#endif
    motion_.set_walk_velocity_step(WALK_V, 0.0f, yaw_cmd, STEP_H);
}
