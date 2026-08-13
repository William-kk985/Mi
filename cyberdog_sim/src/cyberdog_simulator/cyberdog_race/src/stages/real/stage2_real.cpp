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
constexpr float PITCH_S2    = 0.06f;    // 走路轻微低头 ~3.4° (2026-08-14 用户要求, 不破限安全)
constexpr float ENTER_DIST_M = 0.92f;   // 开场衔接前进 0.92m
constexpr float SLIDE_DIST_M = 2.8f;    // 每轮侧移 2.8m
constexpr float FWD_GAP_M    = 1.05f;   // 轮间前进衔接 1.05m (2026-08-14: 1.0→1.05 用户要求)
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
    slide_yaw_ref_   = sensor_.abs_yaw;
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
        slide_yaw_ref_  = sensor_.abs_yaw;   // 锁航向基准 (2026-08-14 漂移补偿)
        traveled_       = 0.0f;
        ball_confirm_   = 0;
        hit_this_round_ = false;
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        phase_ = Phase::SLIDE;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S2Stage] 轮%d %s侧移%.1fm 找球中 (航向锁%.2frad)\n",
                round_ + 1, slide_dir() > 0 ? "左" : "右", SLIDE_DIST_M, slide_yaw_ref_);
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
            // 前进也锁航向 (2026-08-14: 侧移后前进会偏, 同基准slide_yaw_ref_)
            float yaw_err = norm_yaw(slide_yaw_ref_ - sensor_.abs_yaw);
            if (std::abs(yaw_err) < 0.05f) yaw_err = 0.0f;
            float yaw_cmd = std::max(-0.4f, std::min(0.4f, yaw_err * 0.5f));
            motion_.set_walk_velocity_pitch(WALK_V, 0.0f, yaw_cmd, PITCH_S2);
        }
        return;
    }

    // ═══ FWD_GAP: 轮间前进 1.05m ═══
    if (phase_ == Phase::FWD_GAP) {
        accumulate();
        if (traveled_ >= FWD_GAP_M) {
            motion_.stop();
            enter_slide();   // round_ 已在上一轮走满时++过
        } else {
            float yaw_err = norm_yaw(slide_yaw_ref_ - sensor_.abs_yaw);
            if (std::abs(yaw_err) < 0.05f) yaw_err = 0.0f;
            float yaw_cmd = std::max(-0.4f, std::min(0.4f, yaw_err * 0.5f));
            motion_.set_walk_velocity_pitch(WALK_V, 0.0f, yaw_cmd, PITCH_S2);
        }
        return;
    }

    // ═══ SLIDE: 侧移走满 2.8m, 途中找球 ═══
    if (phase_ == Phase::SLIDE) {
        // ── 漂移补偿①: 位移投影到期望侧移方向 (2026-08-14) ──
        //   侧移时存在前进/后退漂移, hypot 会多计; 只计侧移分量才准
        float dx = sensor_.odom_x - last_x_;
        float dy = sensor_.odom_y - last_y_;
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        float a = sensor_.abs_yaw;
        // 期望侧移方向单位向量(世界系): dir=+1 为机器人左侧 = 朝向逆时针转90°
        float side_moved = dx * (-std::sin(a)) * slide_dir()
                         + dy * ( std::cos(a)) * slide_dir();
        if (std::abs(side_moved) > 0.25f) side_moved = 0.0f;
        slide_left_ -= side_moved;
        slide_left_ = std::max(0.0f, slide_left_);

        // ── 漂移补偿②: 锁航向 (侧移时身体会转, yaw反馈拉回) ──
        float yaw_err = norm_yaw(slide_yaw_ref_ - sensor_.abs_yaw);
        if (std::abs(yaw_err) < 0.05f) yaw_err = 0.0f;
        float yaw_cmd = std::max(-0.4f, std::min(0.4f, yaw_err * 0.5f));

        // 本轮尚未撞击 → 找球 (连续12帧确认, 防误检)
        if (!hit_this_round_) {
            if (sensor_.ball_found && sensor_.ball_dist <= BALL_MAX_DIST) {
                if (++ball_confirm_ >= SCAN_CONFIRM_FRAMES) {
                    ball_confirm_ = 0;
                    last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
                    traveled_ = 0.0f;
                    impact_x_ = sensor_.odom_x; impact_y_ = sensor_.odom_y;   // 撞击起点
                    impact_yaw_ = sensor_.abs_yaw;
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
            if (round_ < 3) {   // 轮1~3 走满 → 前进衔接
                round_++;
                phase_ = Phase::FWD_GAP;
                traveled_ = 0.0f;
                last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 轮%d 侧移完成, 前进%.2fm衔接\n", round_, FWD_GAP_M);
                fflush(stderr);
#endif
            } else if (round_ == 3) {   // 轮4走满 → 不前进, 直接轮5左移 (2026-08-14)
                round_++;
                enter_slide();
            } else {
                phase_ = Phase::DONE;        // 轮5: 走满直接结束
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 轮5侧移完成, 全部结束 DONE\n");
                fflush(stderr);
#endif
            }
            return;
        }

        // 侧移速度指令 (vel_des.y + yaw航向锁, 轻微低头)
        motion_.set_walk_velocity_pitch(0.0f, SLIDE_V * slide_dir(), yaw_cmd, PITCH_S2);
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
            motion_.set_walk_velocity_pitch(IMPACT_V, 0.0f, ball_yaw, PITCH_S2);
        }
        return;
    }

    // ═══ BACK: 退回 0.2m → 继续走完本轮侧移剩余 (不再找球) ═══
    if (phase_ == Phase::BACK) {
        // ── 回退距离按机器人后退方向投影 (2026-08-14): odom漂移时hypot会多计少计 ──
        float dx = sensor_.odom_x - last_x_;
        float dy = sensor_.odom_y - last_y_;
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        float a = sensor_.abs_yaw;
        float fwd = dx * std::cos(a) + dy * std::sin(a);   // 机器人前向分量
        traveled_ -= fwd;    // 后退时 fwd<0 → traveled_ 增加; 侧漂不计入
        if (traveled_ >= BACK_DIST) {
            motion_.stop();
            hit_this_round_ = true;      // 本轮已撞 → 剩余只走完不找球 (防重撞)
            last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
            traveled_ = 0.0f;
            phase_ = Phase::SLIDE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 退回完成, 继续侧移剩余%.1fm(不再找球)\n", slide_left_);
            fprintf(stderr, "[S2Stage]   撞击段漂移: 距撞击点=%.3fm 航向差=%.2frad\n",
                    std::hypot(sensor_.odom_x - impact_x_, sensor_.odom_y - impact_y_),
                    norm_yaw(sensor_.abs_yaw - impact_yaw_));
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_pitch(-BACK_V, 0.0f, 0.0f, PITCH_S2);
        }
        return;
    }

    if (phase_ == Phase::DONE) done_ = true;
}
