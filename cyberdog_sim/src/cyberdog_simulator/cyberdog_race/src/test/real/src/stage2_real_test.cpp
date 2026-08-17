#include "stage2_real_test.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>

// ═══════════════════════════════════════════════════════════
// Stage2RealTest 真机版 — 第2赛段 (2026-08-16 点位全局闭环版)
// 动作序列与折线版一致(参考 2026-08-16 实测良好数据), 但执行改为:
//   · 目标点从起点+理论航向递推, 每步导航到全局目标点(≤0.05m到位)
//   · 漂移不跨步累积; 撞击中断→原路退回→继续导航同一步
// 折线动作序列版已迁至 stage2_real_test (2026-08-16)
// ═══════════════════════════════════════════════════════════

namespace {

// ═══ 动作参数 ═══
constexpr float WALK_V      = 0.20f;    // 前进速度 m/s
constexpr float SLIDE_V     = 0.15f;    // 侧移速度 m/s
constexpr float IMPACT_V    = 0.35f;    // 撞击前进速度
constexpr float BACK_V      = 0.20f;    // 退回速度
constexpr float PITCH_S2    = 0.06f;    // 轻微低头 ~3.4°
constexpr float TURN_V      = 0.5f;     // 原地转向速度 rad/s
constexpr int   TURN_SETTLE_FRAMES = 30;  // 转向到位停稳 0.3s
constexpr float ARRIVE_DIST = 0.05f;    // 到达目标点判定 ≤5cm (2026-08-16)
constexpr int   STEP_TIMEOUT = 3000;    // 单步超时30s强制下一步 (防odom卡住)

// ═══ 找球/撞击 ═══
constexpr float BALL_MAX_DIST     = 0.6f;   // 球距≤0.6m 才算找到
constexpr float BALL_CENTER_MAX_X = 0.15f;  // 只撞正前方球 |x|≤0.15
constexpr int   SCAN_CONFIRM_FRAMES = 12;   // 连续12帧确认
constexpr int   IMPACT_LOST_FRAMES  = 8;    // 丢球8帧且<30%→放弃
constexpr float IMPACT_MAX       = 0.6f;    // 撞击最大冲刺
constexpr float TOUCH_DIST       = 0.08f;   // 球距≤0.08m连续3帧=触球
constexpr int   MAX_HITS         = 4;       // 全场最多4球
constexpr float HIT_SPOT_GUARD   = 0.5f;    // 撞过的球位置防重半径

// ═══ 动作序列 (2026-08-16 实测良好版, 与折线版一致) ═══
const std::vector<S2TestStep> STEPS = {
    {S2TestKind::FWD,      0.92f},  // 0 开场衔接前进
    {S2TestKind::SLIDE_L,  0.30f},  // 1 左移0.3
    {S2TestKind::TURN_R90, 0.0f},   // 2 右转90°
    {S2TestKind::SLIDE_L,  2.80f},  // 3 左移2.8
    {S2TestKind::SLIDE_L,  0.10f},  // 4 左移0.1
    {S2TestKind::SLIDE_R,  0.10f},  // 5 回来
    {S2TestKind::TURN_180, 0.0f},   // 6 转180°
    {S2TestKind::SLIDE_R,  0.10f},  // 7 右移0.1
    {S2TestKind::SLIDE_L,  0.10f},  // 8 回来
    {S2TestKind::SLIDE_L,  2.80f},  // 9 左移2.8
    {S2TestKind::FWD,      1.20f},  // 10 前进1.2
    {S2TestKind::SLIDE_R,  2.80f},  // 11 右移2.8
    {S2TestKind::SLIDE_R,  0.10f},  // 12 右移0.1
    {S2TestKind::SLIDE_L,  0.10f},  // 13 回来
    {S2TestKind::FWD,      1.20f},  // 14 前进1.2
    {S2TestKind::SLIDE_R,  0.10f},  // 15 右移0.1
    {S2TestKind::SLIDE_L,  0.10f},  // 16 回来
    {S2TestKind::SLIDE_L,  2.80f},  // 17 左移2.8
    {S2TestKind::TURN_R90, 0.0f},   // 18 右转90°
    {S2TestKind::FWD,      3.00f},  // 19 前进3 离场
    {S2TestKind::FWD,      0.10f},  // 20 前进0.1
    {S2TestKind::TURN_L90, 0.0f},   // 21 左转90°
    {S2TestKind::FWD,      0.20f},  // 22 前进0.2
    {S2TestKind::TURN_R90, 0.0f},   // 23 右转90° → 结束
};

// 归一化角度到 [-π, π]
float norm_yaw(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

// 步方向(世界系单位向量), 基于理论SLAM航向
void step_dir(S2TestKind k, float yaw, float& ux, float& uy) {
    if (k == S2TestKind::FWD) {
        ux = std::cos(yaw); uy = std::sin(yaw);
    } else {
        const float sign = (k == S2TestKind::SLIDE_L) ? 1.0f : -1.0f;
        ux = -std::sin(yaw) * sign;
        uy =  std::cos(yaw) * sign;
    }
}

}  // namespace

void Stage2RealTest::init() {
    done_          = false;
    phase_         = Phase::STEP;
    loc_ready_     = false;
    step_idx_      = 0;
    step_start_    = true;
    yaw_slam_      = 0.0f;
    yaw_odom_      = 0.0f;
    target_x_ = target_y_ = 0.0f;
    turn_target_   = 0.0f;
    turn_settle_   = 0;
    step_timeout_  = 0;
    step_time_     = 0;
    ball_confirm_  = 0;
    impact_lost_   = 0;
    impact_goal_   = 0.0f;
    touch_frames_  = 0;
    impact_x_ = impact_y_ = impact_yaw_ = 0.0f;
    back_timeout_  = 0;
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;
    traveled_      = 0.0f;
    yaw_integ_     = 0.0f;
    hit_total_     = 0;
    impact_path_.clear();
    hit_spots_.clear();
    RCLCPP_INFO(rclcpp::get_logger("stage2_real"),
                "[Stage2RealTest] init: 点位全局闭环 (共%zu步, 到位判定%.2fm)", STEPS.size(), ARRIVE_DIST);
}

void Stage2RealTest::enter_impact(float ball_dist) {
    impact_goal_ = std::max(0.08f, ball_dist);
    impact_x_ = sensor_.odom_x;
    impact_y_ = sensor_.odom_y;
    impact_yaw_ = sensor_.abs_yaw;
    impact_path_.clear();
    hit_total_++;
    const float a = sensor_.abs_yaw;
    hit_spots_.emplace_back(sensor_.odom_x + std::cos(a) * ball_dist,
                            sensor_.odom_y + std::sin(a) * ball_dist);
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;
    traveled_ = 0.0f;
    impact_lost_ = 0;
    touch_frames_ = 0;
    back_timeout_ = 0;
    phase_ = Phase::IMPACT;
#ifdef DEBUG_STAGE
    fprintf(stderr, "[S2Stage] 步%d 确认球(dist=%.2fm x=%.2f), 撞击(第%d个, 目标%.2fm)\n",
            step_idx_, ball_dist, sensor_.ball_x, hit_total_, impact_goal_);
    fflush(stderr);
#endif
}

void Stage2RealTest::run() {
    if (done_) return;

    // ── 等定位就绪 (2026-08-16): 首帧锁理论航向+目标点原点 ──
    // ⚠ 必须同时等 odom_yaw_ready: 转向/航向锁用yaw_odom, 未就绪锁0会满幅乱转
    if (!loc_ready_) {
        if ((sensor_.abs_yaw != 0.0f || sensor_.odom_x != 0.0f) && sensor_.odom_yaw_ready) {
            loc_ready_ = true;
            // (2026-08-16 终版): 纯odom同源建系 — 位置与航向同一估计器, 零偏置
            yaw_slam_  = sensor_.yaw_odom;
            yaw_odom_  = sensor_.yaw_odom;
            target_x_  = sensor_.odom_pos_x;
            target_y_  = sensor_.odom_pos_y;   // (2026-08-16 odom同源)
            last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 定位就绪: 起点odom=(%.2f,%.2f) slamYaw=%.2f odomYaw=%.2f\n",
                    sensor_.odom_x, sensor_.odom_y, yaw_slam_, yaw_odom_);
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, PITCH_S2);
            return;
        }
    }

    // ═══ IMPACT: 朝球冲击 (触球判定制, 与折线版一致) ═══
    if (phase_ == Phase::IMPACT) {
        if (impact_path_.empty() ||
            std::hypot(sensor_.odom_x - impact_path_.back().first,
                       sensor_.odom_y - impact_path_.back().second) > 0.05f) {
            impact_path_.emplace_back(sensor_.odom_x, sensor_.odom_y);
        }
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;
        traveled_ += moved;

        if (sensor_.ball_found && sensor_.ball_dist <= TOUCH_DIST) ++touch_frames_;
        else touch_frames_ = 0;
        const bool reached = traveled_ >= impact_goal_ || touch_frames_ >= 3;

        if (sensor_.ball_found) impact_lost_ = 0;
        else ++impact_lost_;
        const bool lost = impact_lost_ >= IMPACT_LOST_FRAMES &&
                          traveled_ < impact_goal_ * 0.3f;

        if (reached || lost || traveled_ >= IMPACT_MAX) {
            motion_.stop();
            touch_frames_ = 0;
            last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
            phase_ = Phase::BACK;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 撞击完成%s, 原路退回\n",
                    touch_frames_ >= 3 ? "(触球)" : (reached ? "(冲到位)" : (lost ? "(丢球放弃)" : "(0.6m保护)")));
            fflush(stderr);
#endif
        } else {
            const float bx = sensor_.ball_found ? sensor_.ball_x : 0.0f;
            const float vy_track = std::max(-0.25f, std::min(0.25f, -bx * 0.6f));
            const float ball_yaw = std::max(-0.20f, std::min(0.20f, -bx * 0.4f));
            motion_.set_walk_velocity_pitch(IMPACT_V, vy_track, ball_yaw, PITCH_S2);
        }
        return;
    }

    // ═══ BACK: 沿撞击轨迹原路退回 (与折线版一致, 航向锁目标=yaw_odom_理论航向) ═══
    if (phase_ == Phase::BACK) {
        const float cx = sensor_.odom_x, cy = sensor_.odom_y;
        while (!impact_path_.empty()) {
            const float tdx = impact_path_.back().first  - cx;
            const float tdy = impact_path_.back().second - cy;
            if (std::hypot(tdx, tdy) <= 0.08f) impact_path_.pop_back();
            else break;
        }
        float tx = impact_x_, ty = impact_y_;
        if (!impact_path_.empty()) { tx = impact_path_.back().first; ty = impact_path_.back().second; }

        const float ddx = tx - cx, ddy = ty - cy;
        const float dist_back = std::hypot(ddx, ddy);
        const float a_slam = sensor_.abs_yaw;
        const float fwd_u = std::cos(a_slam), fwd_v = std::sin(a_slam);
        const float proj_back = -(ddx * fwd_u + ddy * fwd_v);
        const float lat_err   = ddx * (-fwd_v) + ddy * fwd_u;
        const float lat_cmd   = std::max(-0.08f, std::min(0.08f, lat_err * 0.4f));

        const float yaw_err1 = norm_yaw(impact_yaw_ - sensor_.abs_yaw);
        const float yaw_err2 = norm_yaw(yaw_odom_ - sensor_.yaw_odom);
        float yaw_err = yaw_err1 + yaw_err2;
        if (std::abs(yaw_err) < 0.03f) yaw_err = 0.0f;
        yaw_integ_ = std::max(-0.12f, std::min(0.12f, yaw_integ_ + (yaw_err1 + yaw_err2) * 0.02f));
        const float yaw_cmd = std::max(-0.4f, std::min(0.4f, yaw_err * 0.5f + yaw_integ_));

        back_timeout_++;
        const float yaw_left = std::abs(yaw_err1);
        if ((impact_path_.empty() && dist_back <= 0.06f && yaw_left <= 0.06f) || back_timeout_ > 300) {
            motion_.stop();
            impact_path_.clear();
            back_timeout_ = 0;
            phase_ = Phase::STEP;   // 继续导航本步 (目标点不变)
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 回点完成, 继续步%d (剩余距=%.3fm 航向差=%.2frad)\n",
                    step_idx_, dist_back, yaw_left);
            fflush(stderr);
#endif
        } else {
            float vx = -BACK_V;
            if (proj_back < 0.02f) vx = 0.0f;
            if (dist_back <= 0.06f) vx = 0.0f;
            motion_.set_walk_velocity_pitch(vx, lat_cmd, yaw_cmd, PITCH_S2);
        }
        return;
    }

    // ═══ STEP: 点位导航 ═══
    if (step_idx_ >= (int)STEPS.size()) {
        motion_.stop();
        phase_ = Phase::DONE;
        done_  = true;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S2Stage] 全部动作完成 DONE\n");
        fflush(stderr);
#endif
        return;
    }

    const S2TestStep& s = STEPS[step_idx_];

    // ── 步起点: 递推目标点/理论航向 ──
    if (step_start_) {
        step_start_ = false;
        step_timeout_ = 0;
        step_time_ = 0;
        yaw_integ_ = 0.0f;
        ball_confirm_ = 0;
        if (s.kind == S2TestKind::TURN_R90)      { yaw_slam_ = norm_yaw(yaw_slam_ - M_PI_2); yaw_odom_ = norm_yaw(yaw_odom_ - M_PI_2); }
        else if (s.kind == S2TestKind::TURN_L90) { yaw_slam_ = norm_yaw(yaw_slam_ + M_PI_2); yaw_odom_ = norm_yaw(yaw_odom_ + M_PI_2); }
        else if (s.kind == S2TestKind::TURN_180) { yaw_slam_ = norm_yaw(yaw_slam_ + M_PI);    yaw_odom_ = norm_yaw(yaw_odom_ + M_PI); }
        else {
            float ux, uy;
            step_dir(s.kind, yaw_slam_, ux, uy);
            target_x_ += s.dist * ux;
            target_y_ += s.dist * uy;
        }
        turn_target_ = yaw_odom_;
        turn_settle_ = 0;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S2Stage] 步%d 开始: %s %.2fm (目标=%.2f,%.2f 理论yaw=%.2f/%.2f)\n",
                step_idx_,
                s.kind == S2TestKind::FWD ? "前进" :
                s.kind == S2TestKind::SLIDE_L ? "左移" :
                s.kind == S2TestKind::SLIDE_R ? "右移" :
                s.kind == S2TestKind::TURN_R90 ? "右转90°" :
                s.kind == S2TestKind::TURN_L90 ? "左转90°" : "转180°",
                s.dist, target_x_, target_y_, yaw_slam_, yaw_odom_);
        fflush(stderr);
#endif
    }

    // ── 转向步 ──
    if (s.kind == S2TestKind::TURN_R90 || s.kind == S2TestKind::TURN_L90 || s.kind == S2TestKind::TURN_180) {
        if (turn_settle_ > 0) {
            if (++turn_settle_ >= TURN_SETTLE_FRAMES) {
                step_idx_++;
                step_start_ = true;
                turn_settle_ = 0;
            } else {
                motion_.stop();
            }
            return;
        }
        const float err = norm_yaw(turn_target_ - sensor_.yaw_odom);
        if (std::abs(err) < 0.04f) {
            turn_settle_ = 1;
            motion_.stop();
        } else {
            const float cmd = std::max(-TURN_V, std::min(TURN_V, err * 2.0f));
            motion_.set_walk_velocity_pitch(0.0f, 0.0f, cmd, PITCH_S2);
        }
        return;
    }

    // ── 移动步: 导航到全局目标点 ──
    // 找球 (与折线版一致)
    if (hit_total_ < MAX_HITS && sensor_.ball_found &&
        sensor_.ball_dist <= BALL_MAX_DIST &&
        std::abs(sensor_.ball_x) <= BALL_CENTER_MAX_X) {
        const float a = sensor_.abs_yaw;
        const float bx = sensor_.odom_x + std::cos(a) * sensor_.ball_dist;
        const float by = sensor_.odom_y + std::sin(a) * sensor_.ball_dist;
        bool near_old = false;
        for (auto& sp : hit_spots_) {
            if (std::hypot(bx - sp.first, by - sp.second) < HIT_SPOT_GUARD) { near_old = true; break; }
        }
        if (!near_old) {
            if (++ball_confirm_ >= SCAN_CONFIRM_FRAMES) {
                ball_confirm_ = 0;
                enter_impact(sensor_.ball_dist);
                return;
            }
        } else {
            ball_confirm_ = 0;
        }
    } else {
        ball_confirm_ = 0;
    }

    // ── 到位判定 (2026-08-16 双保险): odom距目标≤5cm 且 指令积分≥90% ──
    //   ① odom虚大(实测侧向+17%)→ 指令积分没满, 继续走够实际距离
    //   ② odom漏记 → 指令满但位置没到, 继续以位置为准
    //   (2026-08-16 终版: 位置用odom_pos, 与航向同源)
    const float ddx = target_x_ - sensor_.odom_pos_x;
    const float ddy = target_y_ - sensor_.odom_pos_y;
    const float dist_go = std::hypot(ddx, ddy);
    step_time_++;
    step_timeout_++;
    const float v_step = (s.kind == S2TestKind::FWD) ? WALK_V : SLIDE_V;
    const float cmd_dist = v_step * step_time_ * 0.01f;   // 指令积分累计
    const bool arrived = (dist_go <= ARRIVE_DIST) && (cmd_dist >= s.dist * 0.90f);
    if (arrived || step_timeout_ > STEP_TIMEOUT) {
        motion_.stop();
        step_idx_++;
        step_start_ = true;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S2Stage] 步%d 到位 (剩余=%.3fm%s)\n",
                step_idx_ - 1, dist_go, step_timeout_ > STEP_TIMEOUT ? ", 超时跳过" : "");
        fflush(stderr);
#endif
        return;
    }

    // ── 垂直全局目标线闭环 + 航向锁 ──
    float ux, uy;
    step_dir(s.kind, yaw_slam_, ux, uy);
    const float along = ddx * ux + ddy * uy;         // 沿步方向剩余(>0继续)
    const float lat   = ddx * (-uy) + ddy * ux;      // 垂直偏差(左正)
    const float lat_cmd = std::max(-0.15f, std::min(0.15f, -lat * 0.6f));

    const float yaw_err_raw = norm_yaw(yaw_odom_ - sensor_.yaw_odom);
    float yaw_err = yaw_err_raw;
    if (std::abs(yaw_err) < 0.03f) yaw_err = 0.0f;
    yaw_integ_ = std::max(-0.25f, std::min(0.25f, yaw_integ_ + yaw_err_raw * 0.02f));
    const float yaw_cmd = std::max(-0.5f, std::min(0.5f, yaw_err * 1.0f + yaw_integ_));

    if (s.kind == S2TestKind::FWD) {
        // 接近目标减速: 剩余<0.3m 按比例降速
        float vx = WALK_V;
        if (along < 0.30f) vx = std::max(0.08f, WALK_V * along / 0.30f);
        motion_.set_walk_velocity_pitch(vx, lat_cmd, yaw_cmd, PITCH_S2);
    } else {
        const float sign = (s.kind == S2TestKind::SLIDE_L) ? 1.0f : -1.0f;
        float vy = SLIDE_V * sign;
        if (along < 0.30f) vy = std::copysign(std::max(0.06f, SLIDE_V * along / 0.30f), sign);
        motion_.set_walk_velocity_pitch(lat_cmd, vy, yaw_cmd, PITCH_S2);
    }
}
