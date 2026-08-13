#include "cyberdog_race/stages/real/stage2_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage2Real 真机版 — 第2赛段 (2026-08-14 侧移扫球重构)
// 流程: 前进0.92m → 轮1左/轮2右/轮3左/轮4右 各侧移2.8m(边走边找球)
//       轮间前进0.75m衔接; 轮4走满直接结束(不前进0.75)
// 全程不转向: 侧移走 303 vel_des.y (机器人系, y正=左假设, 反了改 SLIDE_LEFT_SIGN)
// 找球: 球距≤0.7m 连续确认12帧 → 中断侧移朝球冲击(距离闭环0.12m/0.5m保护)
//       → 退回0.2m → 继续走完本轮侧移剩余(不再找球, 防漏扫+防重撞) → 前进0.75进下一轮
// 撞击完成不跳段: 每轮2.8m全覆盖, 解决"撞击后某个方向不查球"的漏扫问题 (2026-08-14)
// 方向反馈不用 abs_yaw: 全程无转向 (2026-08-14)
// ═══════════════════════════════════════════════════════════

namespace {

// ═══ Stage2 动作参数 (侧移扫球, 2026-08-14 重构) ═══
constexpr float WALK_V      = 0.30f;    // 前进速度 m/s
constexpr float SLIDE_V     = 0.25f;    // 侧移速度 m/s (侧移比前进慢, 2026-08-14 先试0.25)
constexpr float IMPACT_V    = 0.45f;    // 撞击前进速度
constexpr float BACK_V      = 0.24f;    // 退回速度 m/s
constexpr float STEP_H      = 0.17f;    // 步高
constexpr float ENTER_DIST_M = 0.92f;   // 开场衔接前进 0.92m
constexpr float SLIDE_DIST_M = 2.8f;    // 每轮侧移 2.8m
constexpr float FWD_GAP_M    = 0.75f;   // 轮间前进衔接 0.75m (轮4后不前进)
constexpr float BALL_MAX_DIST    = 0.7f;  // 球距≤0.7m 才算找到
constexpr float IMPACT_DIST      = 0.12f; // 撞击到位: 球距<0.12m (距离闭环)
constexpr float IMPACT_MAX       = 0.5f;  // 撞击最大冲刺 0.5m (球丢失保护)
constexpr float BACK_DIST        = 0.2f;  // 撞击后退回 0.2m
constexpr int   SCAN_CONFIRM_FRAMES = 12; // 球连续确认12帧(~0.12s)才算数, 防误检
// ★ 侧移方向符号: 303 vel_des.y, ROS惯例 y正=左。若真机实测相反 → 改成 -1.0f
constexpr float SLIDE_LEFT_SIGN = +1.0f;

}  // namespace

void Stage2Real::init() {
    done_            = false;
    round_           = 0;
    ball_confirm_    = 0;
    hit_this_round_  = false;
    slide_left_      = 0.0f;
    last_x_          = sensor_.odom_x;
    last_y_          = sensor_.odom_y;
    traveled_        = 0.0f;
    phase_           = Phase::FWD0;   // 开场衔接: 前进0.92m (2026-08-14)
    RCLCPP_INFO(rclcpp::get_logger("stage2_real"),
                "[Stage2Real] init: 前进%.2fm → 侧移扫球×4轮(左/右/左/右, 每轮%.1fm+轮间%.1fm)",
                ENTER_DIST_M, SLIDE_DIST_M, FWD_GAP_M);
}

void Stage2Real::run() {
    if (done_) return;

    // ── 通用: 子段累计位移 (odom, 跳变保护; 全程不转向直接hypot) ──
    auto accumulate = [&]() -> float {
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x;
        last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;
        traveled_ += moved;
        return moved;
    };

    // ── 轮次侧移方向: 轮1左 轮2右 轮3左 轮4右 ──
    auto slide_dir = [&]() {
        return (round_ % 2 == 0) ? SLIDE_LEFT_SIGN : -SLIDE_LEFT_SIGN;
    };

    // ── 进入下一轮侧移 ──
    auto enter_slide = [&]() {
        slide_left_     = SLIDE_DIST_M;
        traveled_       = 0.0f;
        ball_confirm_   = 0;
        hit_this_round_ = false;
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        phase_ = Phase::SLIDE;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S2Stage] 轮%d %s侧移%.1fm 找球中\n",
                round_ + 1, slide_dir() > 0 ? "左" : "右", SLIDE_DIST_M);
        fflush(stderr);
#endif
    };

    // ═══ FWD0: 开场衔接前进 0.92m ═══
    if (phase_ == Phase::FWD0) {
        accumulate();
        if (traveled_ >= ENTER_DIST_M) {
            motion_.stop();
            round_ = 0;
            enter_slide();
        } else {
            motion_.set_walk_velocity_step(WALK_V, 0.0f, 0.0f, STEP_H);
        }
        return;
    }

    // ═══ FWD_GAP: 轮间前进 0.75m ═══
    if (phase_ == Phase::FWD_GAP) {
        accumulate();
        if (traveled_ >= FWD_GAP_M) {
            motion_.stop();
            enter_slide();   // round_ 已在上一轮走满时++过
        } else {
            motion_.set_walk_velocity_step(WALK_V, 0.0f, 0.0f, STEP_H);
        }
        return;
    }

    // ═══ SLIDE: 侧移走满 2.8m, 途中找球 ═══
    if (phase_ == Phase::SLIDE) {
        slide_left_ -= accumulate();
        slide_left_ = std::max(0.0f, slide_left_);

        // 本轮尚未撞击 → 找球 (连续12帧确认, 防误检)
        if (!hit_this_round_) {
            if (sensor_.ball_found && sensor_.ball_dist <= BALL_MAX_DIST) {
                if (++ball_confirm_ >= SCAN_CONFIRM_FRAMES) {
                    ball_confirm_ = 0;
                    last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
                    traveled_ = 0.0f;
                    phase_ = Phase::IMPACT;
#ifdef DEBUG_STAGE
                    fprintf(stderr, "[S2Stage] 轮%d 确认球(dist=%.2fm), 中断侧移朝球冲击\n",
                            round_ + 1, sensor_.ball_dist);
                    fflush(stderr);
#endif
                    return;
                }
            } else {
                ball_confirm_ = 0;
            }
        }

        // 侧移走满 → 下一段
        if (slide_left_ <= 0.0f) {
            motion_.stop();
            if (round_ < 3) {
                round_++;                    // 走满才进下一轮
                phase_ = Phase::FWD_GAP;
                traveled_ = 0.0f;
                last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 轮%d 侧移完成, 前进%.2fm衔接\n", round_, FWD_GAP_M);
                fflush(stderr);
#endif
            } else {
                phase_ = Phase::DONE;        // 轮4: 走满直接结束, 不前进0.75
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 轮4侧移完成, 全部结束 DONE\n");
                fflush(stderr);
#endif
            }
            return;
        }

        // 侧移速度指令 (vel_des.y, 身体始终朝前不转向)
        motion_.set_walk_velocity_step(0.0f, SLIDE_V * slide_dir(), 0.0f, STEP_H);
        return;
    }

    // ═══ IMPACT: 朝球冲击 (距离闭环, 0.12m到位 / 0.5m保护) ═══
    if (phase_ == Phase::IMPACT) {
        accumulate();
        bool reached = sensor_.ball_found && sensor_.ball_dist > 0.01f &&
                       sensor_.ball_dist < IMPACT_DIST;
        if (reached || traveled_ >= IMPACT_MAX) {
            motion_.stop();
            last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
            traveled_ = 0.0f;
            phase_ = Phase::BACK;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 撞击完成%s, 退回0.2m\n",
                    reached ? "(球距<0.12m)" : "(0.5m保护)");
            fflush(stderr);
#endif
        } else {
            float bx = sensor_.ball_found ? sensor_.ball_x : 0.0f;
            float ball_yaw = std::max(-0.25f, std::min(0.25f, -bx * 0.5f));
            motion_.set_walk_velocity_step(IMPACT_V, 0.0f, ball_yaw, STEP_H);
        }
        return;
    }

    // ═══ BACK: 退回 0.2m → 继续走完本轮侧移剩余 (不再找球) ═══
    if (phase_ == Phase::BACK) {
        accumulate();
        if (traveled_ >= BACK_DIST) {
            motion_.stop();
            hit_this_round_ = true;      // 本轮已撞 → 剩余只走完不找球 (防重撞)
            last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
            traveled_ = 0.0f;
            phase_ = Phase::SLIDE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 退回完成, 继续侧移剩余%.1fm(不再找球)\n", slide_left_);
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(-BACK_V, 0.0f, 0.0f, STEP_H);
        }
        return;
    }

    if (phase_ == Phase::DONE) done_ = true;
}
