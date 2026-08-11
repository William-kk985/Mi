#include "cyberdog_race/stages/real/stage2_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage2Real 真机版 — 第2赛段
// 右转3° → 走1m → 左转90° → 走3.05m → 右转90°
// → 找球扫描: 左转45°停2秒 / 右转135°停2秒
//   每个角度: 识别到橙色球→前进0.2m再退回; 没有→2秒后转下一个角度
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
constexpr float TURN3_DEG  = -90.0f;   // 动作5: 右转 90°
constexpr float SCAN1_DEG  = +45.0f;   // 扫描位1: 左转 45°
constexpr float SCAN2_DEG  = -90.0f;   // 扫描位2: 右转 90° (从+45°到-45°, 相对基准左右45°对称; 2026-08-12 135°→90°)
constexpr int   SCAN_WAIT_FRAMES = 200; // 每角度停 2 秒 (100Hz)
constexpr float SCAN_POKE_DIST   = 0.2f; // 找到球 前进 0.2m
constexpr float BALL_MAX_DIST    = 0.8f; // 橙色球距离 ≤0.8m 才算找到 (2026-08-12)
constexpr float TURN_SPEED = 0.60f;    // 转向速度 rad/s (test14 同款, +左转)

}  // namespace

void Stage2Real::init() {
    done_          = false;
    start_yaw_     = sensor_.abs_yaw;
    turn_base_yaw_ = sensor_.abs_yaw;
    fwd_ref_yaw_   = sensor_.abs_yaw;
    turn_guard_    = 0;
    wait_frames_   = 0;
    phase_         = Phase::TURN1;
    last_x_        = sensor_.odom_x;
    last_y_        = sensor_.odom_y;
    traveled_      = 0.0f;
    RCLCPP_INFO(rclcpp::get_logger("stage2_real"),
                "[Stage2Real] init: 右转%.0f°→走%.1fm→左转%.0f°→走%.1fm→右转%.0f°→扫描找球",
                TURN1_DEG, DIST1_M, TURN2_DEG, DIST2_M, TURN3_DEG);
}

void Stage2Real::run() {
    if (done_) return;

    // ── 转向: TURN1/2/3 + 扫描转向 SCAN1/2_TURN (相对进入时的朝向) ──
    if (phase_ == Phase::TURN1 || phase_ == Phase::TURN2 || phase_ == Phase::TURN3 ||
        phase_ == Phase::SCAN1_TURN || phase_ == Phase::SCAN2_TURN) {
        float deg = (phase_ == Phase::TURN1)      ? TURN1_DEG :
                    (phase_ == Phase::TURN2)      ? TURN2_DEG :
                    (phase_ == Phase::TURN3)      ? TURN3_DEG :
                    (phase_ == Phase::SCAN1_TURN) ? SCAN1_DEG : SCAN2_DEG;
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
            } else if (phase_ == Phase::TURN3) { // → 扫描位1 (左转45°)
                phase_ = Phase::SCAN1_TURN;
                turn_guard_    = 0;
                turn_base_yaw_ = sensor_.abs_yaw;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 右转90°完成, 扫描左转45°\n");
                fflush(stderr);
#endif
            } else if (phase_ == Phase::SCAN1_TURN) { // → 扫描位1 停2秒
                phase_ = Phase::SCAN1_WAIT;
                wait_frames_ = 0;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 扫描位1(左转45°)停2秒找球\n");
                fflush(stderr);
#endif
            } else {                             // SCAN2_TURN → 扫描位2 停2秒
                phase_ = Phase::SCAN2_WAIT;
                wait_frames_ = 0;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 扫描位2(右转90°)停2秒找球\n");
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

    // ── 扫描停 2 秒找球 ──
    if (phase_ == Phase::SCAN1_WAIT || phase_ == Phase::SCAN2_WAIT) {
        motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);   // 原地停
        bool is_scan1 = (phase_ == Phase::SCAN1_WAIT);
        if (sensor_.ball_found && sensor_.ball_dist <= BALL_MAX_DIST) {   // 球距≤0.8m 才算
            phase_ = is_scan1 ? Phase::SCAN1_ACT : Phase::SCAN2_ACT;
            last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
            traveled_ = 0.0f;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 扫描位%d 发现橙色球(dist=%.2fm≤%.1f)! 前进%.1fm\n",
                    is_scan1 ? 1 : 2, sensor_.ball_dist, BALL_MAX_DIST, SCAN_POKE_DIST);
            fflush(stderr);
#endif
        } else if (++wait_frames_ >= SCAN_WAIT_FRAMES) {   // 2秒无球 → 下一位
            if (is_scan1) {
                phase_ = Phase::SCAN2_TURN;
                turn_guard_    = 0;
                turn_base_yaw_ = sensor_.abs_yaw;
            } else {
                phase_ = Phase::DONE;
            }
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 扫描位%d 2秒无球, %s\n",
                    is_scan1 ? 1 : 2, is_scan1 ? "右转90°" : "DONE");
            fflush(stderr);
#endif
        }
        return;
    }

    // ── 找到球: 前进 0.2m ──
    if (phase_ == Phase::SCAN1_ACT || phase_ == Phase::SCAN2_ACT) {
        bool is_scan1 = (phase_ == Phase::SCAN1_ACT);
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;
        traveled_ += moved;
        if (traveled_ >= SCAN_POKE_DIST) {
            motion_.stop();
            phase_ = is_scan1 ? Phase::SCAN1_BACK : Phase::SCAN2_BACK;
            last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
            traveled_ = 0.0f;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 前进%.1fm完成, 退回\n", SCAN_POKE_DIST);
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(WALK_V, 0.0f, 0.0f, STEP_H);
        }
        return;
    }

    // ── 退回 0.2m ──
    if (phase_ == Phase::SCAN1_BACK || phase_ == Phase::SCAN2_BACK) {
        bool is_scan1 = (phase_ == Phase::SCAN1_BACK);
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;
        traveled_ += moved;
        if (traveled_ >= SCAN_POKE_DIST) {
            motion_.stop();
            if (is_scan1) {
                phase_ = Phase::SCAN2_TURN;
                turn_guard_    = 0;
                turn_base_yaw_ = sensor_.abs_yaw;
            } else {
                phase_ = Phase::DONE;
            }
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 退回完成, %s\n", is_scan1 ? "右转90°" : "DONE");
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(-WALK_V * 0.8f, 0.0f, 0.0f, STEP_H);  // 后退
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
            fprintf(stderr, "[S2Stage] 走%.1fm完成, 右转90°\n", DIST2_M);
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
