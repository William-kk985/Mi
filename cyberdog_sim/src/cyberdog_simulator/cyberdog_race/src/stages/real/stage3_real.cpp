#include "cyberdog_race/stages/real/stage3_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage3Real 真机版 — 第3赛段: 破限低头前进
// 移植 test19 pitch_unlock_test (真机 ✅ 14°低头保持走满0.5m, 2026-08-08)
// 原理: 走路 pitch 被 gait WrapRange(±0.1rad) 夹死, 夹持系数乘
//   (fabs(vx)*x_effect_scale_pos+1.0); 默认 x_effect_scale_pos=-0.55(缩小范围)
//   → LCM 7668 interface_request 设成 +30 → 走路时 pitch 限位放大 7~11 倍
//   → ±0.25 命令穿透 WrapRange, 低头保持!
// ⚠ 该参数同时放大 vel_cmd_scale_(1)/(2) (y/yaw 速度限), 只直行, 完成必须复原 -0.55
// 真机约定: 正值=低头 (2026-08-08 舵机方向确认)
// ═══════════════════════════════════════════════════════════

namespace {
constexpr float  PITCH         = 0.25f;    // 低头 0.25 rad (正值=低头)
constexpr float  SPEED         = 0.20f;    // 前进 0.20 m/s (触发 x_effect_scale_pos 放大)
constexpr float  GOAL_DIST     = 0.5f;     // 走 0.5m (test19 同款)
constexpr float  STEP_H        = 0.17f;    // 步高
constexpr double SCALE_HACK    = 30.0;     // x_effect_scale_pos 破限放大值
constexpr double SCALE_RESTORE = -0.55;    // 默认值 (cyberdog2-ctrl-user-parameters.yaml)
}  // namespace

void Stage3Real::init() {
    done_       = false;
    loc_ready_  = false;
    unlock_set_ = false;
    traveled_   = 0.0f;
    phase_      = Phase::WAIT_READY;

    // ── 站起 (与 Stage1 同款已验证序列) ──
    motion_.locomotion();
    bool svc_ready = motion_.wait_motion_result_ready(5);
    motion_.stand();
    rclcpp::sleep_for(std::chrono::seconds(3));
#ifdef DEBUG_SENSOR
    fprintf(stderr, "[S3S] 站起: 服务%s odom=(%.2f,%.2f) absYaw=%.2f\n",
            svc_ready ? "✅就绪" : "❌超时", sensor_.odom_x, sensor_.odom_y, sensor_.abs_yaw);
    fflush(stderr);
#endif
    motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);   // 预热原地踏步
    RCLCPP_INFO(rclcpp::get_logger("stage3_real"),
                "[Stage3Real] init: 破限低头%.2frad 走%.1fm @%.2fm/s", PITCH, GOAL_DIST, SPEED);
}

void Stage3Real::run() {
    if (done_) return;

    // ── ① 等定位就绪再开始 (init 在 spin 前, 回调没跑 absYaw 恒0, 同Stage1) ──
    if (phase_ == Phase::WAIT_READY) {
        if (sensor_.abs_yaw != 0.0f || sensor_.odom_x != 0.0f) {
            last_x_   = sensor_.odom_x;
            last_y_   = sensor_.odom_y;
            traveled_ = 0.0f;
            // 破限开关: LCM 7668 直改 RT 板参数 x_effect_scale_pos=+30
            motion_.set_user_param_double_lcm("x_effect_scale_pos", SCALE_HACK);
            unlock_set_ = true;
            phase_ = Phase::LOW_WALK;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3Stage] 定位就绪 odom=(%.2f,%.2f), 破限参数+30, 低头前进\n",
                    sensor_.odom_x, sensor_.odom_y);
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);   // 原地踏步等定位
            return;
        }
    }

    // ── ② 破限低头前进, 走满 GOAL_DIST ──
    if (phase_ == Phase::LOW_WALK) {
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x;
        last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;   // 跳变保护
        traveled_ += moved;

        if (traveled_ >= GOAL_DIST) {
            // ★ 复原破限参数 + 回正 + 停
            if (unlock_set_) {
                motion_.set_user_param_double_lcm("x_effect_scale_pos", SCALE_RESTORE);
                unlock_set_ = false;
            }
            motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);   // 回正
            motion_.stop();
            phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3Stage] 破限低头走满 %.2fm, 复原参数, DONE (pitch_map=%.1f°)\n",
                    traveled_, sensor_.pitch_map * 180.0f / M_PI);
            fflush(stderr);
#endif
            return;
        }

        motion_.set_walk_velocity_pitch(SPEED, 0.0f, 0.0f, PITCH);   // 303前进+低头
#ifdef DEBUG_SENSOR
        static int dbg_ = 0;
        if (++dbg_ % 10 == 0) {
            fprintf(stderr, "[S3S] pitch_map=%.1f° 走%.2fm odom=(%.2f,%.2f)\n",
                    sensor_.pitch_map * 180.0f / M_PI, traveled_, sensor_.odom_x, sensor_.odom_y);
            fflush(stderr);
        }
#endif
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; return; }
}
