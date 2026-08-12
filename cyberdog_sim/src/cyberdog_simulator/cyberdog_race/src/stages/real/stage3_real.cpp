#include "cyberdog_race/stages/real/stage3_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage3Real 真机版 — 第3赛段: 视觉巡线 (两边黄色赛道)
// 检测: LaneDetector(on_rgb, /image RGB) → lane_offset/curvature/valid
// 控制: 纯视觉 PD (真机 IMU yaw 恒0, 不做IMU回正)
//   yaw_cmd = -(KP*offset + KD*d_offset), 偏右(offset>0)→右转回中
//   (set_walk_velocity_step yaw 正=左转)
// 弯道: curvature>60 减速 + 增益提高; 丢线: 保持上次yaw慢速直行
// 结束: 累计位移 ≥ GOAL_DIST (2026-08-12 可调)
// ═══════════════════════════════════════════════════════════

namespace {
constexpr float WALK_V        = 0.30f;    // 直道速度 m/s
constexpr float CURVE_V       = 0.18f;    // 弯道速度 m/s
constexpr float STEP_H        = 0.17f;    // 步高
constexpr float KP_YAW        = 0.6f;     // 视觉P增益 (仿真stage1同款)
constexpr float KD_YAW        = 0.10f;    // 视觉D增益
constexpr float CURVE_THRESH  = 60.0f;    // 曲率>60 判弯道 (仿真同款)
constexpr float GOAL_DIST     = 4.0f;     // 巡线总里程 m (2026-08-12 可调!)
constexpr int   LANE_LOST_LIM = 120;      // 丢线保持帧数 (约1.2s@100Hz)
}  // namespace

void Stage3Real::init() {
    done_         = false;
    loc_ready_    = false;
    traveled_     = 0.0f;
    prev_offset_  = 0.0f;
    last_yaw_cmd_ = 0.0f;
    lane_lost_    = 0;
    phase_        = Phase::WAIT_READY;

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
                "[Stage3Real] init: 视觉巡线 直道%.2f 弯道%.2f 里程%.1fm", WALK_V, CURVE_V, GOAL_DIST);
}

void Stage3Real::run() {
    if (done_) return;

    // ── ① 等定位就绪再开始 (init 在 spin 前, 回调没跑 absYaw 恒0, 同Stage1) ──
    if (phase_ == Phase::WAIT_READY) {
        if (sensor_.abs_yaw != 0.0f || sensor_.odom_x != 0.0f) {
            last_x_       = sensor_.odom_x;
            last_y_       = sensor_.odom_y;
            traveled_     = 0.0f;
            prev_offset_  = sensor_.lane_offset;
            last_yaw_cmd_ = 0.0f;
            lane_lost_    = 0;
            phase_ = Phase::LANE_FOLLOW;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3Stage] 定位就绪 odom=(%.2f,%.2f), 开始巡线\n",
                    sensor_.odom_x, sensor_.odom_y);
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);   // 原地踏步等定位
            return;
        }
    }

    // ── ② 视觉巡线 ──
    if (phase_ == Phase::LANE_FOLLOW) {
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x;
        last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;   // 跳变保护
        traveled_ += moved;

        if (traveled_ >= GOAL_DIST) {      // 走满 → 结束
            motion_.stop();
            phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3Stage] 巡线走满 %.2fm, DONE\n", traveled_);
            fflush(stderr);
#endif
            return;
        }

        // 丢线保护: 保持上次 yaw 慢速直行; 丢线过久 → 更慢
        if (!sensor_.lane_valid) {
            lane_lost_++;
            float hold = (lane_lost_ > LANE_LOST_LIM) ? 0.10f : WALK_V * 0.6f;
            motion_.set_walk_velocity_step(hold, 0.0f, last_yaw_cmd_, STEP_H);
#ifdef DEBUG_STAGE
            if (lane_lost_ % 30 == 0) {
                fprintf(stderr, "[S3Stage] 丢线 %d帧, 保持前进 hold=%.2f\n", lane_lost_, hold);
                fflush(stderr);
            }
#endif
            return;
        }
        lane_lost_ = 0;

        // 视觉 PD: 弯道提高增益 + 减速
        float d_offset = sensor_.lane_offset - prev_offset_;
        prev_offset_   = sensor_.lane_offset;
        bool curve = sensor_.lane_curvature > CURVE_THRESH;
        float kp = curve ? KP_YAW * 1.3f : KP_YAW;
        float kd = curve ? KD_YAW * 1.2f : KD_YAW;
        float yaw_cmd = -(kp * sensor_.lane_offset + kd * d_offset);
        yaw_cmd = std::max(-0.5f, std::min(0.5f, yaw_cmd));
        last_yaw_cmd_ = yaw_cmd;
        float speed = curve ? CURVE_V : WALK_V;

        motion_.set_walk_velocity_step(speed, 0.0f, yaw_cmd, STEP_H);
#ifdef DEBUG_SENSOR
        static int dbg_ = 0;
        if (++dbg_ % 10 == 0) {
            fprintf(stderr, "[S3S] off=%.2f curv=%.0f valid=%d v=%.2f yaw=%.2f 走%.2fm\n",
                    sensor_.lane_offset, sensor_.lane_curvature, (int)sensor_.lane_valid,
                    speed, yaw_cmd, traveled_);
            fflush(stderr);
        }
#endif
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; return; }
}
