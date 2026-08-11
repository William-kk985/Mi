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
    start_yaw_   = sensor_.abs_yaw;      // ⚠ 转向必须用 abs_yaw(地图绝对朝向), IMU yaw 真机恒0 (2026-08-11)
    last_x_      = sensor_.odom_x;
    last_y_      = sensor_.odom_y;

    // ── 站起: 完全移植 behavior test 已验证动作序列 ──
    //   构造函数 Motion init 已做 recovery+locomotion; 这里与 run_test() 开头一致再切一次
    //   forward_test 同款: stand()(111官方站立) → 等站稳
    motion_.locomotion();                                    // 切行走模式(run_test 开头同款)
    bool svc_ready = motion_.wait_motion_result_ready(5);    // 等 MotionResultCmd 服务就绪
    motion_.stand();                                         // RECOVERYSTAND 官方站立
    rclcpp::sleep_for(std::chrono::seconds(3));              // 等真正站起(服务异步)
#ifdef DEBUG_SENSOR
    fprintf(stderr, "[S1S] 站起: 服务%s odom=(%.2f,%.2f) yaw=%.2f absYaw=%.2f\n",
            svc_ready ? "✅就绪" : "❌超时", sensor_.odom_x, sensor_.odom_y, sensor_.yaw, sensor_.abs_yaw);
#endif

    motion_.set_walk_velocity(0.0f, 0.0f, 0.0f);  // 预热原地踏步(测试 march 同款; 步高默认0.15)
    motion_.set_body_pitch(-0.10f);                          // 微微抬头(真机负值=抬头)
    RCLCPP_INFO(rclcpp::get_logger("stage1_real"), "[Stage1Real] init: 步高0.15 前进%.1fm", GOAL_DIST);
}

void Stage1Real::run() {
    if (done_) return;

    // ── ② 原地转 90° ─────────────────────────────────────
    if (phase_ == Phase::TURN) {
        float yaw_err = norm_yaw(start_yaw_ + TURN_YAW - sensor_.abs_yaw);   // ⚠ abs_yaw 闭环
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S1Stage] TURN: absYaw=%.2f 目标=%.2f err=%.2f\n",
                sensor_.abs_yaw, start_yaw_ + TURN_YAW, yaw_err);
#endif
        if (std::abs(yaw_err) < 0.05f) {                     // 转到位
            motion_.stop();
            phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S1Stage] 转弯完成, DONE\n");
#endif
        } else {
            float turn = std::max(0.10f, std::min(0.45f, std::abs(yaw_err) * 0.6f));
            motion_.set_walk_velocity(0.0f, 0.0f, yaw_err > 0 ? turn : -turn);
        }
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; return; }

    // ── ① 前进 6m 巡线 ───────────────────────────────────
    float dist = std::hypot(sensor_.odom_x - start_x_, sensor_.odom_y - start_y_);
    if (dist >= GOAL_DIST) {                                 // 走完 6m
        motion_.stop();
        phase_ = Phase::TURN;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S1Stage] 前进 %.2fm 完成, 转 90°\n", dist);
#endif
        return;
    }

    // 卡住检测: 位移过小连续 N 帧 → 冲刺脱困(加速冲过石板)
    float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;
#ifdef DEBUG_SENSOR
    static int dbg_ = 0;
    if (++dbg_ % 10 == 0)
        fprintf(stderr, "[S1S] odom=(%.2f,%.2f) yaw=%.2f absYaw=%.2f dist=%.2f moved=%.4f stuck=%d rush=%d\n",
                sensor_.odom_x, sensor_.odom_y, sensor_.yaw, sensor_.abs_yaw, dist, moved, stuck_, rush_);
#endif

    if (rush_ > 0) {                                         // 冲刺中
        motion_.set_walk_velocity(RUSH_V, 0.0f, 0.0f);
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

    // ── 纯直行前进 (暂不上视觉巡线, 2026-08-11 用户要求; 后续再加回 lane_detector) ──
    // 用测试验证过的 set_walk_velocity (303, 步高默认 0.15), 用 abs_yaw 回正防走偏 (IMU yaw 真机恒0)
    float yaw_cmd = std::max(-0.5f, std::min(0.5f, -sensor_.abs_yaw * 0.8f));
#ifdef DEBUG_MOTION
    static int dbg_m_ = 0;
    if (++dbg_m_ % 10 == 0)
        fprintf(stderr, "[S1M] cmd v=(%.2f,0,%.2f) 步高0.15\n", WALK_V, yaw_cmd);
#endif
    motion_.set_walk_velocity(WALK_V, 0.0f, yaw_cmd);
}
