#include "cyberdog_race/stages/real/stage1_real.hpp"
#include "cyberdog_race/debug_config.hpp"   // ★ 必须显式 include, DEBUG_MOTION/SENSOR/STAGE 宏 (2026-08-11 缺这个导致打印没编译!)
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage1Real 真机版 — 石径探路
// 步高 0.15 | 前进 6m | 视觉巡线 + 里程计 + IMU 转90°
// 真机接口: set_walk_velocity_step = 303 WALK_USERTROT + step_height
//   (motion_servo_cmd.step_height 字段, 真机正确步高通道 2026-08-08 确认)
// ═══════════════════════════════════════════════════════════

namespace {

constexpr float STEP_H        = 0.17f;   // 步高 0.17 (石板磕碰测试, 2026-08-11 0.15→0.17)
constexpr float WALK_V        = 0.30f;   // 正常前进速度 m/s
constexpr float RUSH_V        = 0.50f;   // 卡住冲刺速度
constexpr float GOAL_DIST     = 3.7f;    // 前进 3.7m (2026-08-12 校准 3.75→3.65→3.7)
constexpr float TURN_YAW      = M_PI / 2.0f;  // 目标转角 90° (test14 相对转向)
constexpr float TURN_SPEED    = 0.60f;   // 转向速度 rad/s (test14 同款, +0.6=左转, 2026-08-07 验证)
constexpr float KP_YAW        = 0.8f;    // 视觉比例
constexpr float KD_YAW        = 0.3f;    // 视觉微分
constexpr float IMU_WEIGHT    = 0.3f;    // IMU 回正权重(视觉为主)
constexpr float STUCK_DIST    = 0.01f;   // 卡住判定位移阈值 (m)
constexpr int   STUCK_THRESH  = 30;      // 卡住帧数 (2026-08-12 恢复默认)
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
    traveled_    = 0.0f;
    turn_guard_  = 0;
    loc_ready_   = false;   // 定位待就绪: 构造函数在 spin 前, global_to_robot 恒0, 需在 run() 里等

    // ── 站起: 完全移植 behavior test 已验证动作序列 ──
    //   构造函数 Motion init 已做 recovery+locomotion; 这里与 run_test() 开头一致再切一次
    //   forward_test 同款: stand()(111官方站立) → 等站稳
    motion_.locomotion();                                    // 切行走模式(run_test 开头同款)
    bool svc_ready = motion_.wait_motion_result_ready(5);    // 等 MotionResultCmd 服务就绪
    motion_.stand();                                         // RECOVERYSTAND 官方站立
    rclcpp::sleep_for(std::chrono::seconds(3));              // 等真正站起(服务异步)
#ifdef DEBUG_SENSOR
    fprintf(stderr, "[S1S] 站起: 服务%s odom=(%.2f,%.2f) yaw=%.2f absYaw=%.2f tof=%.2f\n",
            svc_ready ? "✅就绪" : "❌超时", sensor_.odom_x, sensor_.odom_y, sensor_.yaw, sensor_.abs_yaw, sensor_.tof_clearance);
    fflush(stderr);
#endif

    motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);  // 预热原地踏步(步高0.17)
    motion_.set_body_pitch(-0.10f);                          // 微微抬头(真机负值=抬头)
    RCLCPP_INFO(rclcpp::get_logger("stage1_real"), "[Stage1Real] init: 步高%.2f 前进%.1fm", STEP_H, GOAL_DIST);
}

void Stage1Real::run() {
    if (done_) return;

    // ── ② 最后原地转 90°: test14 相对转向同款 (abs_yaw 闭环, 真机验证误差~2.4%) ──
    //   README: 反馈必须用 abs_yaw(global_to_robot.rpy[2]), IMU yaw 真机恒0 别用
    if (phase_ == Phase::TURN) {
        float yaw_err = norm_yaw(start_yaw_ + TURN_YAW - sensor_.abs_yaw);
        bool turn_done = (turn_guard_ > 20) && (std::abs(yaw_err) < 0.05f);  // 至少转0.2s防跳变
        turn_guard_++;
        if (turn_done) {                     // 转到位 (test14: err<0.05)
            motion_.stop();
            phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S1Stage] 转向完成 err=%.2f, DONE\n", yaw_err);
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(0.0f, 0.0f, TURN_SPEED, STEP_H);  // 固定左转0.6 (test14同款)
#ifdef DEBUG_STAGE
            if (turn_guard_ % 20 == 0) {
                fprintf(stderr, "[S1Stage] 转向中 err=%.2f absYaw=%.2f 目标=%.2f\n",
                        yaw_err, sensor_.abs_yaw, start_yaw_ + TURN_YAW);
                fflush(stderr);
            }
#endif
        }
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; return; }

    // ── ① 等定位就绪再开始前进 ──
    // ⚠ 真正根因 (2026-08-12): init 在构造函数(spin前)跑, global_to_robot 回调没执行 → absYaw 恒0
    //   必须在 run()(spin后)里等定位就绪, 再记录起点, 否则 start_yaw_=0 实际1.5 → 回正疯狂转向
    if (!loc_ready_) {
        if (sensor_.abs_yaw != 0.0f || sensor_.odom_x != 0.0f) {
            loc_ready_  = true;
            start_x_    = sensor_.odom_x;
            start_y_    = sensor_.odom_y;
            start_yaw_  = sensor_.abs_yaw;
            last_x_     = sensor_.odom_x;
            last_y_     = sensor_.odom_y;
            traveled_   = 0.0f;
            turn_guard_ = 0;
#ifdef DEBUG_SENSOR
            fprintf(stderr, "[S1S] 定位就绪: odom=(%.2f,%.2f) absYaw=%.2f 开始前进\n",
                    sensor_.odom_x, sensor_.odom_y, sensor_.abs_yaw);
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);  // 原地踏步等定位
#ifdef DEBUG_SENSOR
            static int wait_dbg_ = 0;
            if (++wait_dbg_ % 50 == 0) {
                fprintf(stderr, "[S1S] 等待定位... absYaw=%.2f\n", sensor_.abs_yaw);
                fflush(stderr);
            }
#endif
            return;
        }
    }

    // ── ① 前进 6m (累计位移判定, 防定位跳变 2026-08-11) ──
    // ⚠ 之前用绝对位置差, 站起时 global_to_robot 定位跳变 → dist 瞬间≥6m → 误判走完
    float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;
    if (moved > 0.25f) moved = 0.0f;   // 跳变保护: 单帧>25cm 视为定位跳变, 忽略
    traveled_ += moved;
    float dist = traveled_;

    if (dist >= GOAL_DIST) {                                 // 累计走完 6m
        motion_.stop();
        phase_ = Phase::TURN;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S1Stage] 前进 %.2fm 完成, 转 90°\n", dist);
        fflush(stderr);
#endif
        return;
    }
#ifdef DEBUG_SENSOR
    static int dbg_ = 0;
    if (++dbg_ % 10 == 0) {
        fprintf(stderr, "[S1S] odom=(%.2f,%.2f) yaw=%.2f absYaw=%.2f dist=%.2f moved=%.4f tof=%.2f elev=%.2f stuck=%d rush=%d\n",
                sensor_.odom_x, sensor_.odom_y, sensor_.yaw, sensor_.abs_yaw, dist, moved,
                sensor_.tof_clearance, sensor_.tof_elev_max, stuck_, rush_);
        fflush(stderr);
    }
#endif

    if (rush_ > 0) {                                         // 冲刺中
        motion_.set_walk_velocity_step(RUSH_V, 0.0f, 0.0f, STEP_H);
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

    // ── ① 先直行前进 6m (最后才转向) ──
    // set_walk_velocity_step: 303 + 自定义步高 STEP_H(0.17)
    // 回正用 abs_yaw 相对站起时朝向(start_yaw_), 防走偏 (IMU yaw 恒0别用)
    float yaw_cmd = std::max(-0.5f, std::min(0.5f, -(sensor_.abs_yaw - start_yaw_) * 0.8f));
#ifdef DEBUG_MOTION
    static int dbg_m_ = 0;
    if (++dbg_m_ % 10 == 0) {
        fprintf(stderr, "[S1M] cmd v=(%.2f,0,%.2f) 步高%.2f\n", WALK_V, yaw_cmd, STEP_H);
        fflush(stderr);
    }
#endif
    motion_.set_walk_velocity_step(WALK_V, 0.0f, yaw_cmd, STEP_H);
}
