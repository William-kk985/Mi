#include "cyberdog_race/stages/real/stage2_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>

// ═══════════════════════════════════════════════════════════
// Stage2Real 真机版 — 第2赛段 (2026-08-15 用户重定流程)
// 前进0.92 → 左移0.3 → 右转90° → 左移3 → 左移0.1回 → 转180° →
// 前进1 → 右移0.1回 → 左移3 → 前进1 → 右移3 → 右移0.1回 →
// 前进1 → 左移3 → 右转90° → 前进3 → DONE
// 移动中找正前方球(|x|≤0.15)撞击; 全场最多4球, 撞过位置防重;
// 撞击触球判定+轨迹原路退回 (旧侧移扫球逻辑已迁至 stage2_real_test)
// ═══════════════════════════════════════════════════════════

namespace {

// ═══ 动作参数 ═══
constexpr float WALK_V      = 0.20f;    // 前进速度 m/s
constexpr float SLIDE_V     = 0.15f;    // 侧移速度 m/s
constexpr float SLIDE_L_FWD_COMP = 0.04f;  // (2026-08-18 回退到昨晚08-17版本: 0.02→0.04; 用户: 昨晚版本方向不错)
constexpr float SLIDE_R_FWD_COMP = 0.05f;  // (2026-08-18 回退: 0.03→0.05)
constexpr float FWD_LAT_COMP   = 0.00f;  // (2026-08-18 回退: -0.02→0.00 恢复昨晚版本; 08-17注释: 偏左→0.03归零试, 再偏左给负值)
constexpr float IMPACT_V    = 0.45f;    // 撞击前进速度 (2026-08-15 0.35→0.45 太保守加点力)
constexpr float BACK_V      = 0.20f;    // 退回速度
constexpr float PITCH_S2    = 0.06f;    // 轻微低头 ~3.4°
constexpr float TURN_V      = 0.5f;     // 原地转向速度 rad/s
constexpr int   TURN_SETTLE_FRAMES = 30;  // 转向到位停稳 0.3s (10ms循环)
constexpr float TURN_EXTRA_RAD = -2.0f * M_PI / 180.0f;  // (2026-08-17 用户: 单步物理欠转2°, 恢复-2°; 08-18回退到昨晚版本)

// ═══ 找球/撞击 ═══
constexpr float BALL_MAX_DIST     = 0.6f;   // 球距≤0.6m 才算找到
constexpr float BALL_CENTER_MAX_X = 0.15f;  // 只撞正前方球 |x|≤0.15 (2026-08-15 宁漏不撞)
constexpr int   SCAN_CONFIRM_FRAMES = 12;   // 连续12帧确认
constexpr int   IMPACT_LOST_FRAMES  = 8;    // 丢球8帧才放弃 (2026-08-15 6→8 不再保守)
constexpr float IMPACT_MAX       = 0.6f;    // 撞击最大冲刺 0.6m (2026-08-15 0.5→0.6 不再保守)
constexpr float TOUCH_DIST       = 0.08f;   // 球距≤0.08m连续3帧=触球 (2026-08-15 0.06→0.08 不再保守)
constexpr int   MAX_HITS         = 4;       // 全场最多4球 (2026-08-15 防误识别多撞)
constexpr float HIT_SPOT_GUARD   = 0.5f;    // 撞过的球位置防重半径 m

// ═══ 动作序列 (用户 2026-08-15 重定; 2026-08-16 删所有0.1m短横移, 步14前进1.2→1.0) ═══
const std::vector<S2Step> STEPS = {
    {S2Kind::FWD,      0.95f},  // 0 开场衔接前进 (2026-08-17 用户: 0.92→0.95)
    {S2Kind::SLIDE_L,  0.30f},  // 1 左移0.3
    {S2Kind::TURN_R90, 0.0f},   // 2 右转90°
    {S2Kind::SLIDE_L,  2.80f},  // 3 左移2.8 撞球 (2026-08-15 3→2.8)
    {S2Kind::TURN_180, 0.0f},   // 4 转180°
    {S2Kind::SLIDE_L,  2.80f},  // 5 左移2.8 撞球
    {S2Kind::FWD,      1.20f},  // 6 前进1.20 换行 (2026-08-18 用户: 1.15→1.20)
    {S2Kind::SLIDE_R,  2.70f},  // 7 右移2.7 撞球 (2026-08-17 用户: 2.8→2.7)
    {S2Kind::FWD,      1.12f},  // 8 前进1.12 换行 (2026-08-17 用户: 1.20→1.12)
    {S2Kind::SLIDE_L,  2.80f},  // 9 左移2.8 撞球
    {S2Kind::TURN_R90, 0.0f},   // 10 右转90°
    {S2Kind::FWD,      3.10f},  // 11 前进3.1 离场 (2026-08-18 用户: 3.00→3.10)
    {S2Kind::FWD,      0.10f},  // 12 前进0.1 (2026-08-16 用户追加)
    {S2Kind::TURN_L90, 0.0f},   // 13 左转90°
    {S2Kind::FWD,      0.50f},  // 14 前进0.5 (2026-08-17 用户: 0.2→0.5)
    {S2Kind::TURN_R90, 0.0f},   // 15 右转90°
    {S2Kind::FWD,      0.60f},  // 16 前进0.6 (2026-08-17 用户: 0.4→0.6)
};

// 归一化角度到 [-π, π]
float norm_yaw(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

}  // namespace

void Stage2Real::init() {
    done_             = false;
    phase_            = Phase::STEP;
    step_idx_         = 0;
    step_start_       = true;
    step_left_        = 0.0f;
    traveled_         = 0.0f;
    side_cmd_accum_   = 0.0f;
    side_odom_accum_  = 0.0f;
    yaw_integ_        = 0.0f;
    ball_confirm_     = 0;
    turn_target_      = 0.0f;
    turn_settle_      = 0;
    impact_lost_      = 0;
    impact_goal_      = 0.0f;
    touch_frames_     = 0;
    impact_x_ = impact_y_ = impact_yaw_ = 0.0f;
    back_timeout_     = 0;
    hit_total_        = 0;
    fix_phase_        = 0;
    fix_frames_       = 0;
    fix_step_yaw_     = 0.0f;
    fix_step_dir_     = 0.0f;
    fix_step_dist_    = 0.0f;
    fix_lat_integ_    = 0.0f;
    hold_frames_      = 0;
    anchor_x_         = 0.0f;
    anchor_y_         = 0.0f;
    anchor_traveled_  = 0.0f;
    impact_path_.clear();
    hit_spots_.clear();
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;
    start_abs_yaw_ = sensor_.abs_yaw;   // (2026-08-18 DONE诊断: 量化最终朝向偏转)
    RCLCPP_INFO(rclcpp::get_logger("stage2_real"),
                "[Stage2Real] init: 折线扫场流程 (前进0.92→左移0.3→右转90°→左移3→...→前进3), 共%zu步",
                STEPS.size());
}

// ── 撞击回点后站稳1s (2026-08-17) ──
void Stage2Real::do_hold() {
    if (++hold_frames_ < 100) {   // 1s
        motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, PITCH_S2);
        return;
    }
    hold_frames_ = 0;
    // (2026-08-17 恢复重起步: 清撞击/退回残留状态, 防“恢复后第一步猛偏”)
    //   last_x_/last_y_停在撞击终点→回STEP第一帧moved=撞击距离被计进步距;
    //   旧锚点+yaw_integ_残留→恢复第一帧vx_lock/yaw_cmd猛拉
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;
    anchor_x_ = sensor_.odom_x;
    anchor_y_ = sensor_.odom_y;
    anchor_traveled_ = side_odom_accum_;
    yaw_integ_ = 0.0f;
    fwd_integ_ = 0.0f;
    last_yaw_err_ = 0.0f;
    phase_ = Phase::STEP;   // 继续当前步剩余
}

// ── 步末矫正: 0=修航向 → 1=修横向 → 完成进下一步 (2026-08-16) ──
void Stage2Real::do_step_fix() {
    ++fix_frames_;

    // 本步理想终点 (起点 + 本步方向 × 本步距离)
    const float ix = step_start_x_ + std::cos(fix_step_dir_) * fix_step_dist_;
    const float iy = step_start_y_ + std::sin(fix_step_dir_) * fix_step_dist_;
    const float wx = sensor_.odom_x - ix;   // 世界系偏差
    const float wy = sensor_.odom_y - iy;
    const float a = sensor_.abs_yaw;
    // 偏差投影到机体系: 前向 u = cos(a)*wx + sin(a)*wy; 左向 v = -sin(a)*wx + cos(a)*wy
    const float v_left = -std::sin(a) * wx + std::cos(a) * wy;

    const float yaw_err = norm_yaw(fix_step_yaw_ - a);
    const bool  yaw_ok  = std::abs(yaw_err) < 0.03f;   // (2026-08-18 回退0.02→0.03 恢复昨晚版本; 08-17: 差一点不修减少来回折腾)
    const bool  lat_ok  = std::abs(v_left) < 0.04f;    // (2026-08-17 0.02→0.04: 偏差4cm内直接过)
    const bool  timeout = fix_frames_ > 100;           // (2026-08-17 250→100: 最多修1秒, 用户嫌矫正走太多)

    if (fix_phase_ == 0) {
        if (yaw_ok || timeout) { fix_phase_ = 1; fix_frames_ = 0; fix_lat_integ_ = 0.0f; motion_.stop(); return; }
        const float yv = std::max(-0.20f, std::min(0.20f, yaw_err * 1.2f));   // (2026-08-17 0.30→0.20 柔和)
        motion_.set_walk_velocity_pitch(0.0f, 0.0f, yv, PITCH_S2);
        return;
    }
    // fix_phase_ == 1: 修横向 (世界偏差 → 机体左向速度拉回; 2026-08-17 加积分修得更干净)
    if (lat_ok || timeout) {
        motion_.stop();
        fix_phase_  = 0;
        fix_frames_ = 0;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S2Fix] 步%d 矫正完成 偏差lat=%.3fm yaw=%.2frad integ=%.3f\n",
                step_idx_, v_left, yaw_err, fix_lat_integ_);
        fflush(stderr);
#endif
        step_idx_++;
        step_start_ = true;
        phase_ = Phase::STEP;
        return;
    }
    fix_lat_integ_ = std::max(-0.10f, std::min(0.10f, fix_lat_integ_ + v_left * 0.01f));
    const float vy = std::max(-0.10f, std::min(0.10f, -v_left * 0.8f - fix_lat_integ_ * 0.4f));   // (2026-08-17 0.15→0.10 柔和防来回)
    motion_.set_walk_velocity_pitch(0.0f, vy, 0.0f, PITCH_S2);
}

// ── 进入撞击: 记录起点/轨迹/球位置, 全场计数 ──
void Stage2Real::enter_impact(float ball_dist) {
    impact_goal_ = std::max(0.08f, ball_dist);   // 不再-0.05: 视觉可能是球面距 (2026-08-15)
    impact_x_ = sensor_.odom_x;
    impact_y_ = sensor_.odom_y;
    impact_yaw_ = sensor_.abs_yaw;
    impact_path_.clear();
    hit_total_++;                                // 全场撞球计数
    // 撞过的球世界位置 (防二次撞击): 球在正前方, 沿 SLAM 航向投影
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

void Stage2Real::run() {
    if (done_) return;

    // ═══ FIX: 步末矫正 (2026-08-16 用户: 每小段走完矫正再继续) ═══
    if (phase_ == Phase::FIX) { do_step_fix(); return; }

    // ═══ HOLD: 撞击回点后原地踏步稳定 (2026-08-17 替代0.1m小步恢复作用) ═══
    if (phase_ == Phase::HOLD) { do_hold(); return; }

    // ═══ IMPACT: 朝球冲击 (触球判定制) ═══
    if (phase_ == Phase::IMPACT) {
        // 轨迹记录 (每~5cm)
        if (impact_path_.empty() ||
            std::hypot(sensor_.odom_x - impact_path_.back().first,
                       sensor_.odom_y - impact_path_.back().second) > 0.05f) {
            impact_path_.emplace_back(sensor_.odom_x, sensor_.odom_y);
        }
        // 位移累计
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;
        traveled_ += moved;

        // 触球判定: 球距≤0.06m 连续3帧
        if (sensor_.ball_found && sensor_.ball_dist <= TOUCH_DIST) ++touch_frames_;
        else touch_frames_ = 0;
        const bool reached = traveled_ >= impact_goal_ || touch_frames_ >= 3;

        // 丢球计数
        if (sensor_.ball_found) impact_lost_ = 0;
        else ++impact_lost_;
        const bool lost = impact_lost_ >= IMPACT_LOST_FRAMES &&
                          traveled_ < impact_goal_ * 0.3f;   // (2026-08-15 0.5→0.3: 更晚放弃, 不保守)

        if (reached || lost || traveled_ >= IMPACT_MAX) {
            motion_.stop();
            touch_frames_ = 0;
            last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
            phase_ = Phase::BACK;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 撞击完成%s, 原路退回\n",
                    touch_frames_ >= 3 ? "(触球)" : (reached ? "(冲到位)" : (lost ? "(丢球放弃)" : "(0.5m保护)")));
            fflush(stderr);
#endif
        } else {
            // 球x闭环: vy横移追球 + yaw小转向
            const float bx = sensor_.ball_found ? sensor_.ball_x : 0.0f;
            const float vy_track = std::max(-0.25f, std::min(0.25f, -bx * 0.6f));
            const float ball_yaw = std::max(-0.20f, std::min(0.20f, -bx * 0.4f));
            motion_.set_walk_velocity_pitch(IMPACT_V, vy_track, ball_yaw, PITCH_S2);
        }
        return;
    }

    // ═══ BACK: 沿撞击轨迹原路退回 ═══
    if (phase_ == Phase::BACK) {
        const float cx = sensor_.odom_x, cy = sensor_.odom_y;
        // 弹出已到达的轨迹点
        while (!impact_path_.empty()) {
            const float tdx = impact_path_.back().first  - cx;
            const float tdy = impact_path_.back().second - cy;
            if (std::hypot(tdx, tdy) <= 0.08f) impact_path_.pop_back();
            else break;
        }
        // 目标: 轨迹尾; 轨迹空 → 撞击前点
        float tx = impact_x_, ty = impact_y_;
        if (!impact_path_.empty()) { tx = impact_path_.back().first; ty = impact_path_.back().second; }

        const float ddx = tx - cx, ddy = ty - cy;
        const float dist_back = std::hypot(ddx, ddy);
        const float a_slam = sensor_.abs_yaw;
        const float fwd_u = std::cos(a_slam), fwd_v = std::sin(a_slam);
        const float proj_back = -(ddx * fwd_u + ddy * fwd_v);
        const float lat_err   = ddx * (-fwd_v) + ddy * fwd_u;
        const float lat_cmd   = std::max(-0.08f, std::min(0.08f, lat_err * 0.4f));

        // 航向双锁 (2026-08-16 收紧: P0.5→0.8 死区0.03→0.01; 2026-08-15 曾柔化 P1.0→0.5)
        const float yaw_err1 = norm_yaw(impact_yaw_ - sensor_.abs_yaw);
        const float yaw_err2 = norm_yaw(step_yaw_odom_ - sensor_.yaw_odom);
        float yaw_err = yaw_err1 + yaw_err2;
        if (std::abs(yaw_err) < 0.01f) yaw_err = 0.0f;   // (2026-08-16 死区0.03→0.01: 防残差积累致斜)
        yaw_integ_ = std::max(-0.12f, std::min(0.12f, yaw_integ_ + (yaw_err1 + yaw_err2) * 0.02f));
        const float yaw_cmd = std::max(-0.4f, std::min(0.4f, yaw_err * 0.8f + yaw_integ_));  // (2026-08-16 P 0.5→0.8)

        back_timeout_++;
        const float yaw_left = std::abs(yaw_err1);
        // 结束: 轨迹清空 + 回撞击前点≤0.04m + 航向差≤0.02rad (超时300帧兜底) (2026-08-16 0.06→0.04/0.02 防残差积累)
        if ((impact_path_.empty() && dist_back <= 0.04f && yaw_left <= 0.02f) || back_timeout_ > 300) {
            motion_.stop();
            impact_path_.clear();
            back_timeout_ = 0;
            hold_frames_ = 0;   // (2026-08-17 站稳1s再继续, 替代0.1m小步恢复作用)
            phase_ = Phase::HOLD;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 回点完成, 站稳后继续步%d (剩余距=%.3fm 航向差=%.2frad)\n",
                    step_idx_, dist_back, yaw_left);
            fflush(stderr);
#endif
        } else {
            float vx = -BACK_V;
            if (proj_back < 0.02f) vx = 0.0f;      // 退过头 → 只侧移
            if (dist_back <= 0.04f) vx = 0.0f;     // 位置到位 → 原地转正 (2026-08-16 0.06→0.04)
            motion_.set_walk_velocity_pitch(vx, lat_cmd, yaw_cmd, PITCH_S2);
        }
        return;
    }

    // ═══ STEP: 动作序列执行 ═══
    if (step_idx_ >= (int)STEPS.size()) {
        motion_.stop();
        phase_ = Phase::DONE;
        done_  = true;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S2Stage] 全部动作完成 DONE (起点absYaw=%.2f 终点=%.2f 偏转=%.1f° odom=(%.2f,%.2f))\n",
                start_abs_yaw_, sensor_.abs_yaw,
                (sensor_.abs_yaw - start_abs_yaw_) * 180.0f / 3.14159265f,
                sensor_.odom_x, sensor_.odom_y);
        fflush(stderr);
#endif
        return;
    }

    const S2Step& s = STEPS[step_idx_];

    // ── 步起点初始化 ──
    if (step_start_) {
        step_start_      = false;
        step_yaw_slam_   = sensor_.abs_yaw;
        step_yaw_odom_   = sensor_.yaw_odom;
        step_start_x_    = sensor_.odom_x;
        step_start_y_    = sensor_.odom_y;
        step_left_       = s.dist;
        traveled_        = 0.0f;
        side_cmd_accum_  = 0.0f;
        side_odom_accum_ = 0.0f;
        anchor_x_        = sensor_.odom_x;   // (2026-08-17 段中锚点)
        anchor_y_        = sensor_.odom_y;
        anchor_traveled_ = 0.0f;
        yaw_integ_       = 0.0f;
        last_yaw_err_    = 0.0f;   // (2026-08-16) 新锚误差基准重置; drift_rate_ 跨步保留(机械特性)
        fwd_integ_       = 0.0f;   // (2026-08-16) 前向积分每步清零
        ball_confirm_    = 0;
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        // (2026-08-16) 转向源改 abs_yaw(与Stage1同款): odom yaw与物理航向有固定偏差导致"左移走左前"
        // (2026-08-18 回退昨晚版本: 统一-2°补偿, 去掉左转单独+2°/180°177°)
        if (s.kind == S2Kind::TURN_R90)      turn_target_ = sensor_.abs_yaw - (M_PI_2 + TURN_EXTRA_RAD);
        else if (s.kind == S2Kind::TURN_L90) turn_target_ = sensor_.abs_yaw + (M_PI_2 + TURN_EXTRA_RAD);
        else if (s.kind == S2Kind::TURN_180) turn_target_ = sensor_.abs_yaw + M_PI;   // (2026-08-17 用户: 180°转满, 不用-2°欠转补偿)
        turn_settle_ = 0;
        turn_guard_   = 0;
        adjust_frames_ = 0;
        turn_start_x_  = sensor_.odom_x;   // ADJUST顶回基准 (2026-08-16 原地转向会后退漂移)
        turn_start_y_  = sensor_.odom_y;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S2Stage] 步%d 开始: %s %.2fm (slam_yaw=%.2f odom_yaw=%.2f)\n",
                step_idx_,
                s.kind == S2Kind::FWD ? "前进" :
                s.kind == S2Kind::SLIDE_L ? "左移" :
                s.kind == S2Kind::SLIDE_R ? "右移" :
                s.kind == S2Kind::TURN_R90 ? "右转90°" :
                s.kind == S2Kind::TURN_L90 ? "左转90°" : "转180°",
                s.dist, step_yaw_slam_, step_yaw_odom_);
        fflush(stderr);
#endif
    }

    // ── 转向步 ──
    if (s.kind == S2Kind::TURN_R90 || s.kind == S2Kind::TURN_L90 || s.kind == S2Kind::TURN_180) {
        if (turn_settle_ > 0) {   // 停稳计数
            if (++turn_settle_ >= TURN_SETTLE_FRAMES) {
                turn_settle_ = 0;
                // ── 停稳复核 (2026-08-16 移植Stage1): odom在快速转向时低估转角, 停稳后误差仍大 → 低速补转 ──
                const float err2 = norm_yaw(turn_target_ - sensor_.abs_yaw);
                if (std::abs(err2) > 0.02f && turn_guard_ < 60) {   // 残余>1.1°, 最多再补0.6s (2026-08-17 0.03→0.02)
                    ++turn_guard_;
                    const float spd = std::max(-0.30f, std::min(0.30f, err2 * 2.0f));
                    motion_.set_walk_velocity_pitch(0.0f, 0.0f, spd, PITCH_S2);
                    return;
                }
                turn_guard_ = 0;
                // ── ADJUST 顶回 (2026-08-16): 原地转向会后退/侧漂, 朝转向起点小步顶回 (历史a635652方案) ──
                const float adx = turn_start_x_ - sensor_.odom_x;
                const float ady = turn_start_y_ - sensor_.odom_y;
                const float adr = std::hypot(adx, ady);
                if (adr > 0.02f && adjust_frames_ < 60) {   // >2cm, 最多0.6s
                    ++adjust_frames_;
                    const float a = sensor_.abs_yaw;
                    const float u = std::cos(a) * adx + std::sin(a) * ady;   // 世界→机体前向
                    const float v = -std::sin(a) * adx + std::cos(a) * ady;  // 世界→机体左向
                    const float ru = std::hypot(u, v);
                    if (ru > 0.01f) {
                        motion_.set_walk_velocity_pitch(0.08f * u / ru, 0.08f * v / ru, 0.0f, PITCH_S2);
                        return;
                    }
                }
                adjust_frames_ = 0;
                step_idx_++;
                step_start_ = true;
            } else {
                motion_.stop();
            }
            return;
        }
        const float err = norm_yaw(turn_target_ - sensor_.abs_yaw);
        if (std::abs(err) < 0.02f) {   // (2026-08-17 0.03→0.02 收紧: 用户转向不够准)
            turn_settle_ = 1;
            motion_.stop();
        } else {
            const float cmd = std::max(-TURN_V, std::min(TURN_V, err * 2.0f));
            motion_.set_walk_velocity_pitch(0.0f, 0.0f, cmd, PITCH_S2);
        }
        return;
    }

    // ── 移动步 (FWD/SLIDE): 找球 → 距离累计 → 指令 ──
    // 找球: 正前方 |x|≤0.15 连续12帧, 全场<4, 撞过位置防重
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

    // 距离累计: 投影到本步方向 + 双源融合
    float dx = sensor_.odom_x - last_x_;
    float dy = sensor_.odom_y - last_y_;
    last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
    float dir_u, dir_v;
    if (s.kind == S2Kind::FWD) {
        dir_u = std::cos(step_yaw_odom_); dir_v = std::sin(step_yaw_odom_);   // (2026-08-16 与航向锁同源, 防SLAM步内漂污染距离计)
    } else {   // 侧移: 左移=机体系y正=航向逆时针90°
        const float sign = (s.kind == S2Kind::SLIDE_L) ? 1.0f : -1.0f;
        dir_u = -std::sin(step_yaw_odom_) * sign;
        dir_v =  std::cos(step_yaw_odom_) * sign;
    }
    float moved = dx * dir_u + dy * dir_v;
    if (std::abs(moved) > 0.25f) moved = 0.0f;
    const float v_step = (s.kind == S2Kind::FWD) ? WALK_V : SLIDE_V;
    side_cmd_accum_  += v_step * 0.01f;
    side_odom_accum_ += moved;
    float eff;
    if (side_cmd_accum_ - side_odom_accum_ < 0.30f)
        eff = std::max(moved, v_step * 0.01f);
    else
        eff = moved;
    eff = std::min(eff, v_step * 0.01f * 1.2f);
    step_left_ -= eff;
    step_left_ = std::max(0.0f, step_left_);

    if (step_left_ <= 0.0f) {
        motion_.stop();
        // ── 步末矫正 (2026-08-16 用户: 每小段走完矫正一下再继续): 航向→横向, 修完进下一步 ──
        if (s.kind == S2Kind::FWD) {
            fix_step_dir_ = step_yaw_slam_;
        } else {
            fix_step_dir_ = step_yaw_slam_ + ((s.kind == S2Kind::SLIDE_L) ? M_PI_2 : -M_PI_2);
        }
        fix_step_yaw_  = step_yaw_slam_;
        fix_step_dist_ = s.dist;
        fix_phase_  = 0;
        fix_frames_ = 0;
        phase_ = Phase::FIX;
        return;
    }

    // 航向锁 (2026-08-17 方案A: 前进段锁yaw_odom; 横移段锁abs_yaw — 侧移时腿里程计yaw假漂→机头斜→"歪的直线")
    const float yaw_err_raw = (s.kind == S2Kind::FWD)
        ? norm_yaw(step_yaw_odom_ - sensor_.yaw_odom)
        : norm_yaw(step_yaw_slam_ - sensor_.abs_yaw);
    float yaw_err = yaw_err_raw;
    if (std::abs(yaw_err) < 0.01f) yaw_err = 0.0f;   // (2026-08-18 回退0.005→0.01 恢复昨晚版本; 08-16: 死区防残差积累致斜)
    yaw_integ_ = std::max(-0.25f, std::min(0.25f, yaw_integ_ + yaw_err_raw * 0.02f));
    // ── yaw 漂移率前馈 (2026-08-16): 提前抵消横移步内的持续漂移; 只前馈yaw, 不加位置积分 ──
    const float err_rate = (yaw_err_raw - last_yaw_err_) * 100.0f;   // 10ms → rad/s
    last_yaw_err_ = yaw_err_raw;
    drift_rate_ += (err_rate - drift_rate_) * 0.02f;                 // 低通 τ≈0.5s
    const float yaw_ff = std::max(-0.25f, std::min(0.25f, drift_rate_ * 1.5f));
    // (2026-08-18 回退昨晚版本: 去掉YAW_COMP开环补偿)
    const float yaw_cmd = std::max(-0.5f, std::min(0.5f, yaw_err * 1.0f + yaw_integ_ + yaw_ff));

    // ── 段中锚点重置 (2026-08-17 替代0.1m小步分段作用): 每走0.7m重锚, 漂移闭环只针对最近0.7m ──
    if (side_odom_accum_ - anchor_traveled_ >= 0.7f) {
        anchor_x_ = sensor_.odom_x; anchor_y_ = sensor_.odom_y;
        anchor_traveled_ = side_odom_accum_;
    }

    if (s.kind == S2Kind::FWD) {
        // 前进: 侧向累计漂移闭环 (2026-08-15: 本帧增量→累计偏差, 增益0.6 限幅±0.15; 2026-08-17 锚点制)
        const float fu = std::cos(step_yaw_slam_), fv = std::sin(step_yaw_slam_);
        const float dev_lat = (sensor_.odom_x - anchor_x_) * (-fv) +
                              (sensor_.odom_y - anchor_y_) * fu;
        const float vy_lock = std::max(-0.15f, std::min(0.15f, -dev_lat * 0.6f));
        // (2026-08-17) 叠加开环左偏补偿: 前进物理右偏 → vy 向左打底
        motion_.set_walk_velocity_pitch(WALK_V,
            std::max(-0.15f, std::min(0.15f, vy_lock + FWD_LAT_COMP)), yaw_cmd, PITCH_S2);
    } else {
        // 侧移: 前向累计漂移闭环 (2026-08-15: 本帧增量→累计偏差, 增益0.6 限幅±0.15; 2026-08-17 锚点制)
        const float fu = std::cos(step_yaw_slam_), fv = std::sin(step_yaw_slam_);
        const float dev_fwd = (sensor_.odom_x - anchor_x_) * fu +
                              (sensor_.odom_y - anchor_y_) * fv;
        // ── 前向积分 (2026-08-16): 闭环自动学习横移角偏 (2026-08-18 回退±0.03→±0.05 恢复昨晚版本)
        fwd_integ_ = std::max(-0.05f, std::min(0.05f, fwd_integ_ + dev_fwd * 0.01f));
        const float vx_lock = std::max(-0.25f, std::min(0.25f, -(dev_fwd * 0.9f + fwd_integ_ * 1.0f)));
        const float slide_sign = (s.kind == S2Kind::SLIDE_L) ? 1.0f : -1.0f;
        // (2026-08-17 左右分开: 左移开环-0.08过度→走左后, 右移+0.08略大→走右前)
        const float fwd_comp = (s.kind == S2Kind::SLIDE_L) ? SLIDE_L_FWD_COMP : SLIDE_R_FWD_COMP;
        float vx_cmd = vx_lock - slide_sign * fwd_comp;
        // (2026-08-18 回退昨晚版本: 去掉cmd_vx钳位)
        static int slide_dbg_ = 0;   // (2026-08-16 无条件诊断: 每100帧一次, 看横移角偏与补偿)
        if (++slide_dbg_ % 100 == 0) {
            fprintf(stderr, "[S2Slide] devF=%.3f integ=%.3f vxLock=%.3f cmd_vx=%.3f yawErr=%.3f yawCmd=%.3f absYaw=%.2f\n",
                    dev_fwd, fwd_integ_, vx_lock, vx_cmd, yaw_err, yaw_cmd, sensor_.abs_yaw);
            fflush(stderr);
        }
        motion_.set_walk_velocity_pitch(vx_cmd, SLIDE_V * slide_sign, yaw_cmd, PITCH_S2);
    }
}
