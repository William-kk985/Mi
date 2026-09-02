#include "cyberdog_race/stages/real/stage3_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage3Real 真机版 — 第3赛段: 写死路径 (替代视觉巡线)
// 路径(相对转向+前进): 直行0.6m → 右转30°前0.5m → 右转40°前0.6m → 右转7°前1.8m
//                      → 左转40°前0.8m → 左转57°前0.8m → 左转93°
// 转向: abs_yaw 闭环(右转=负yaw); 前进: odom 距离闭环 + 航向锁
// 踏步保护: odom 卡死时指令里程兜底 + 转向5s超时
// ═══════════════════════════════════════════════════════════

namespace {
struct PathStep {
    float turn_deg;   // 相对转向角度(正=左转 负=右转)
    float fwd_m;      // 前进距离
};
constexpr PathStep PATH[] = {
    {  0.0f, 0.6f },   // ⓿ 直行0.6m (衔接 Stage2 移交的第一步)
    { -30.0f, 0.5f },   // ① 右转30° 前0.5m
    { -40.0f, 0.6f },   // ② 右转40° 前0.6m
    {  -7.0f, 1.8f },   // ③ 右转7° 前1.8m 长直线
    { +40.0f, 0.8f },   // ④ 左转40° 前0.8m
    { +57.0f, 0.8f },   // ⑤ 左转57° 前0.8m
    { +93.0f, 0.0f },   // ⑥ 左转93° 结束
};
constexpr int   PATH_N      = 7;   // 头部直行0.6m 衔接 Stage2
constexpr float WALK_V      = 0.30f;   // 前进速度 m/s
constexpr float TURN_V      = 0.50f;   // 转向速度 rad/s
constexpr float TURN_DONE   = 0.02f;   // 转向到位误差 rad (对齐 Stage2; 太松会每步欠转累积角度偏)
constexpr int   TURN_SETTLE = 30;      // 转到位停稳帧数 0.3s
constexpr float PITCH_S3    = 0.14f;   // 走路低头 ~8° (pitch 正值=低头)
}  // namespace

void Stage3Real::init() {
    done_      = false;
    step_idx_  = 0;
    target_yaw_ = 0.0f;
    turn_settle_ = 0;
    traveled_   = 0.0f;
    turn_guard_ = 0;
    drift_rate_ = 0.0f;
    last_yaw_err_ = 0.0f;
    cmd_travel_ = 0.0f;    // 踏步保护: 指令里程兜底
    turn_total_ = 0;
    phase_     = Phase::WAIT_READY;

    // ── 站起: 服务起来会站立, 不主动干预 (避免已站立时被误判趴着→强站摔倒); 开跑前等 body_height≥0.23 ──
#ifdef DEBUG_SENSOR
    fprintf(stderr, "[S3S] init: body_h=%.2f (等待服务站起, 不主动干预)\n", sensor_.body_height);
    fflush(stderr);
#endif
    RCLCPP_INFO(rclcpp::get_logger("stage3_real"),
                "[Stage3Real] init: 写死路径 %d 步 (直行0.6+右30+右40+右7+左40+左57+左93)",
                PATH_N);
}

void Stage3Real::run() {
    if (done_) return;

    // ── ① 等定位就绪 (init 在 spin 前, 回调没跑 absYaw 恒0, 同Stage1) ──
    if (phase_ == Phase::WAIT_READY) {
        if ((sensor_.abs_yaw != 0.0f || sensor_.odom_x != 0.0f)
            && sensor_.body_height >= 0.23f) {   // 开跑前必须确认已站立, 防开始就摔
            step_idx_   = 0;
            target_yaw_ = norm_yaw(sensor_.abs_yaw + PATH[0].turn_deg * M_PI / 180.0f);
            turn_settle_ = 0;
            turn_total_ = 0;   // 转向超时计数归零
            phase_ = Phase::TURN;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3Stage] 定位就绪 absYaw=%.2f, 第1步: %s转%.0f°前%.1fm\n",
                    sensor_.abs_yaw,
                    PATH[0].turn_deg >= 0 ? "左" : "右", std::abs(PATH[0].turn_deg), PATH[0].fwd_m);
            fflush(stderr);
#endif
        } else {
#ifdef DEBUG_SENSOR
            static int w_ = 0;
            if (++w_ % 50 == 0) {
                fprintf(stderr, "[S3S] 等待定位... absYaw=%.2f\n", sensor_.abs_yaw);
                fflush(stderr);
            }
#endif
            return;   // 狗保持站立(111)静止等待
        }
    }

    // ── ② 路径步: 先原地转相对角, 再前进 ──
    if (phase_ == Phase::TURN) {
        const float err = norm_yaw(target_yaw_ - sensor_.abs_yaw);
        // ── 转向超时保护: 5 秒转不完强制进 FWD, 防踏步死循环 ──
        if (++turn_total_ > 500) {
            turn_guard_ = 0;
            step_start_x_ = last_x_ = sensor_.odom_x;
            step_start_y_ = last_y_ = sensor_.odom_y;
            step_cos_ = std::cos(target_yaw_);
            step_sin_ = std::sin(target_yaw_);
            traveled_ = 0.0f;
            cmd_travel_ = 0.0f;
            last_yaw_err_ = 0.0f;
            turn_settle_ = 0;
            phase_ = Phase::FWD;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3Stage] ⚠ 第%d步转向超时5s, 强制前进 (absYaw=%.2f 目标=%.2f)\n",
                    step_idx_ + 1, sensor_.abs_yaw, target_yaw_);
            fflush(stderr);
#endif
            return;
        }
        if (std::abs(err) < TURN_DONE) {
            motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, PITCH_S3);
            if (++turn_settle_ >= TURN_SETTLE) {   // 停稳0.3s
                // ── 停稳复核 (与 Stage2 对齐): 停稳后残余误差仍大→低速补转; 阈值收紧防残余累积偏 ──
                if (std::abs(err) > 0.025f && turn_guard_ < 60) {
                    ++turn_guard_;
                    const float spd = std::max(-0.30f, std::min(0.30f, err * 2.0f));
                    motion_.set_walk_velocity_pitch(0.0f, 0.0f, spd, PITCH_S3);
                    return;
                }
                turn_guard_ = 0;
                turn_settle_ = 0;
                // 记本前进步起点与朝向（侧向漂移闭环用）
                step_start_x_ = last_x_ = sensor_.odom_x;
                step_start_y_ = last_y_ = sensor_.odom_y;
                step_cos_ = std::cos(target_yaw_);
                step_sin_ = std::sin(target_yaw_);
                traveled_ = 0.0f;
                cmd_travel_ = 0.0f;    // 指令里程归零
                last_yaw_err_ = 0.0f;
                phase_ = Phase::FWD;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S3Stage] 第%d步转到位, 前进%.1fm\n",
                        step_idx_ + 1, PATH[step_idx_].fwd_m);
                fflush(stderr);
#endif
            }
            return;
        }
        turn_settle_ = 0;
        float yv = std::max(-TURN_V, std::min(TURN_V, err * 1.2f));
        motion_.set_walk_velocity_pitch(0.0f, 0.0f, yv, PITCH_S3);
        return;
    }

    if (phase_ == Phase::FWD) {
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x;
        last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;
        traveled_ += moved;
        // ── 原地踏步保护: odom 卡死/打滑时按指令里程兜底, 防一直踏步不结束 ──
        cmd_travel_ += WALK_V * 0.01f;
        const float eff = std::max(traveled_, cmd_travel_ - 0.20f);   // 指令领先odom>0.2m才起兜底

        if (eff >= PATH[step_idx_].fwd_m) {
            motion_.stop();
            if (++step_idx_ >= PATH_N) {
                // 切换摔修复: 走完先抬平停稳 0.5s 再 DONE (否则 Stage3→4 切换时狗低头+伺服退出→摔)
                phase_ = Phase::SETTLE;
                turn_settle_ = 0;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S3Stage] 全部%d步完成, 收尾抬平停稳\n", PATH_N);
                fflush(stderr);
#endif
            } else {
                target_yaw_ = norm_yaw(target_yaw_ + PATH[step_idx_].turn_deg * M_PI / 180.0f);
                turn_settle_ = 0;
                turn_total_ = 0;   // 转向超时计数归零
                cmd_travel_ = 0.0f;
                phase_ = Phase::TURN;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S3Stage] 第%d步: %s转%.0f°前%.1fm\n",
                        step_idx_ + 1,
                        PATH[step_idx_].turn_deg >= 0 ? "左" : "右",
                        std::abs(PATH[step_idx_].turn_deg), PATH[step_idx_].fwd_m);
                fflush(stderr);
#endif
            }
            return;
        }

        // 前进 + 航向锁(锁本步目标航向) + 侧向漂移闭环
        // 无漂移率前馈 (VIO yaw 抖±2°被放大→长直线左右摆); 死区收紧多修正
        float yaw_err = norm_yaw(target_yaw_ - sensor_.abs_yaw);
        if (std::abs(yaw_err) < 0.015f) yaw_err = 0.0f;
        const float yaw_cmd = std::max(-0.5f, std::min(0.5f, yaw_err * 0.8f));
        // 侧向漂移闭环: 偏离本步直线越远 → 横向速度拉回
        const float dev_lat = (sensor_.odom_x - step_start_x_) * (-step_sin_) +
                              (sensor_.odom_y - step_start_y_) * step_cos_;
        const float vy_lock = std::max(-0.15f, std::min(0.15f, -dev_lat * 0.6f));
        motion_.set_walk_velocity_pitch(WALK_V, vy_lock, yaw_cmd, PITCH_S3);
#ifdef DEBUG_SENSOR
        static int dbg2_ = 0;
        if (++dbg2_ % 10 == 0) {
            fprintf(stderr, "[S3S] 第%d步 %.2f/%.2fm absYaw=%.2f yaw=%.2f\n",
                    step_idx_ + 1, traveled_, PATH[step_idx_].fwd_m,
                    sensor_.abs_yaw, yaw_cmd);
            fflush(stderr);
        }
#endif
        return;
    }

    if (phase_ == Phase::SETTLE) {
        // 303 静止抬平 (保持伺服模式不丢, 身体回正, 再 DONE)
        motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);
        if (++turn_settle_ >= 50) {
            phase_ = Phase::DONE;
            done_ = true;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3Stage] 收尾完成, DONE\n");
            fflush(stderr);
#endif
        }
    }

    if (phase_ == Phase::DONE) done_ = true;
}

