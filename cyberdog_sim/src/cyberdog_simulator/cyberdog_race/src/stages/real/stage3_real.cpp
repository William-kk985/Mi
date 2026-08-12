#include "cyberdog_race/stages/real/stage3_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage3Real 真机版 — 第3赛段: 破限低头 + 视觉检测 (测试形态, 不前进)
// 破限低头: LCM 7668 设 x_effect_scale_pos=+30 放大走路pitch限位
//   → set_walk_velocity_pitch(0,0,0,PITCH) 原地踏步低头保持 (test19 真机✅14°)
// 视觉: LaneDetector(on_rgb /image) 持续检测, Web 标注看两侧黄线
// ⚠ 测试形态: 原地低头不前进, 便于调视觉; 正式巡线待验证 (2026-08-12)
// ⚠ 破限参数同时放大 y/yaw 限位, 本形态不发转向; 手动停止后重启复原
// 真机约定: 正值=低头 (2026-08-08 舵机方向确认)
// ═══════════════════════════════════════════════════════════

namespace {
constexpr float  PITCH         = 0.25f;    // 低头 0.25 rad (正值=低头)
constexpr float  STEP_H        = 0.17f;    // 步高
constexpr double SCALE_HACK    = 30.0;     // x_effect_scale_pos 破限放大值
constexpr double SCALE_RESTORE = -0.55;    // 默认值 (cyberdog2-ctrl-user-parameters.yaml)
}  // namespace

void Stage3Real::init() {
    done_       = false;
    loc_ready_  = false;
    unlock_set_ = false;
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
                "[Stage3Real] init: 破限低头+视觉 测试形态(原地不前进)");
}

void Stage3Real::run() {
    if (done_) return;

    // ── ① 等定位就绪再开始 (init 在 spin 前, 回调没跑 absYaw 恒0, 同Stage1) ──
    if (phase_ == Phase::WAIT_READY) {
        if (sensor_.abs_yaw != 0.0f || sensor_.odom_x != 0.0f) {
            // 破限开关: LCM 7668 直改 RT 板参数 x_effect_scale_pos=+30
            motion_.set_user_param_double_lcm("x_effect_scale_pos", SCALE_HACK);
            unlock_set_ = true;
            phase_ = Phase::LOW_HOLD;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3Stage] 定位就绪 odom=(%.2f,%.2f), 破限+30, 原地低头保持\n",
                    sensor_.odom_x, sensor_.odom_y);
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);   // 原地踏步等定位
            return;
        }
    }

    // ── ② 破限低头原地保持 + 视觉检测 (不前进) ──
    if (phase_ == Phase::LOW_HOLD) {
        // 原地踏步低头: 速度0 + 低头PITCH (破限后无夹持, pitch_map≈14°)
        motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, PITCH);
        // 视觉检测由 on_rgb 持续运行, 这里只打印状态供观察
#ifdef DEBUG_SENSOR
        static int dbg_ = 0;
        if (++dbg_ % 10 == 0) {
            fprintf(stderr, "[S3S] pitch_map=%.1f° off=%.2f curv=%.0f valid=%d | 原地保持\n",
                    sensor_.pitch_map * 180.0f / M_PI, sensor_.lane_offset,
                    sensor_.lane_curvature, (int)sensor_.lane_valid);
            fflush(stderr);
        }
#endif
        return;   // 测试形态: 不前进, 不结束 (保持原地低头+视觉)
    }
}

