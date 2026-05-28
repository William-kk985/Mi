#include "cyberdog_race/stages/stage7.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

void Stage7::init() {
    done_        = false;
    jump_frames_ = 0;
    retry_count_ = 0;
    state_       = State::START;
    fprintf(stderr, "\033[1;35m[Stage7::init] 初始化Stage7，里程计=(%.3f,%.3f)\033[0m\n", 
            sensor_.odom_x, sensor_.odom_y);
}

void Stage7::run() {
    if (done_) return;

    switch (state_) {

    // ── ① 先QpStand站好（RC遥控器标准流程），再触发跳跃 ──
    case State::START: {
        if (jump_frames_ == 0) {
            fprintf(stderr, "\033[1;34m[Stage7 START] 发送QpStand命令...\033[0m\n");
            motion_.stand();  // gamepad x按钮 → QpStand
        }
        jump_frames_++;
        if (jump_frames_ >= 30) {  // 等0.3秒完成QpStand
            state_ = State::PRE_JUMP;
            jump_frames_ = 0;
            fprintf(stderr, "\033[1;34m[Stage7 START] ✓ QpStand完成，准备跳跃\033[0m\n");
        }
        break;
    }

    // ── ② 先发LCM跳命令（use_rc=0时不生效但启动定时器），再切use_rc=1 ──
    case State::PRE_JUMP: {
        if (jump_frames_ == 0) {
            // 第一帧：发LCM跳命令（此时use_rc=0，LCM暂不生效，但lcm_timer已启动）
            jump_start_x_ = sensor_.odom_x;
            jump_start_y_ = sensor_.odom_y;
            fprintf(stderr, "\033[1;35m[Stage7 PRE_JUMP] 🚀 发送LCM跳跃命令 (第%d次)\033[0m\n", retry_count_ + 1);
            motion_.jump();
        }
        jump_frames_++;
        if (jump_frames_ == 5) {
            // 5帧后LCM命令已到达，此时切use_rc=1，lcm_timer已新鲜
            rc_mode_needed_ = true;
        }
        if (jump_frames_ >= 15) {
            state_ = State::JUMP_FORWARD;
            jump_frames_ = 0;
        }
        break;
    }

    // ── ③ 等待跳跃完成 ──
    case State::JUMP_FORWARD: {
        // 持续发RecoveryStand保持LCM定时器存活，防止趴下
        motion_.send_lcm_mode(12);
        jump_frames_++;
        if (jump_frames_ <= 3 || jump_frames_ % 60 == 0) {
            float dx = sensor_.odom_x - jump_start_x_;
            float dy = sensor_.odom_y - jump_start_y_;
            float dist = std::sqrt(dx*dx + dy*dy);
            fprintf(stderr, "\033[1;36m[Stage7 JUMP] frame=%d dist=%.3fm\033[0m\n", jump_frames_, dist);
        }
        // 跳跃轨迹620步×2ms=1.24s + 预蹲500步=1s ≈ 2.5s
        if (jump_frames_ > 250) {
            float dx = sensor_.odom_x - jump_start_x_;
            float dy = sensor_.odom_y - jump_start_y_;
            float dist = std::sqrt(dx*dx + dy*dy);
            fprintf(stderr, "\033[1;32m[Stage7 JUMP] ✓ 结束，移动%.3fm\033[0m\n", dist);
            state_ = State::CHECK_JUMP;
            jump_frames_ = 0;
        }
        break;
    }

    // ── ④ 检查跳跃结果 ──
    case State::CHECK_JUMP: {
        float dx = sensor_.odom_x - jump_start_x_;
        float dy = sensor_.odom_y - jump_start_y_;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (retry_count_ < MAX_RETRY) {
            retry_count_++;
            fprintf(stderr, "\033[1;33m[Stage7] → 跳了%.2fm (第%d次)，连续再跳！\033[0m\n", 
                    dist, retry_count_);
            state_ = State::RESET;
            jump_frames_ = 0;
        } else {
            fprintf(stderr, "\033[1;32m[Stage7] ✓ 连续%d跳完成，最后跳了%.2fm\033[0m\n", 
                    MAX_RETRY + 1, dist);
            state_ = State::FINISH;
        }
        break;
    }

    // ── ⑤ 退出跳跃模式，准备重试 ──
    case State::RESET: {
        // 持续发送RecoveryStand，保持LCM定时器存活，退出kJump3d
        motion_.send_lcm_mode(12);  // kRecoveryStand
        jump_frames_++;
        if (jump_frames_ == 1) {
            fprintf(stderr, "\033[1;34m[Stage7 RESET] 退出跳跃模式...\033[0m\n");
        }
        if (jump_frames_ >= 50) {  // 等0.5秒让状态切换+稳定
            state_ = State::PRE_JUMP;
            jump_frames_ = 0;
            fprintf(stderr, "\033[1;34m[Stage7 RESET] ✓ 准备再次跳跃\033[0m\n");
        }
        break;
    }

    // ── ⑥ 结束 ──
    case State::FINISH: {
        // 持续发送LCM RecoveryStand(12)保持站立，避免LCM定时器过期趴下
        motion_.send_lcm_mode(12);  // kRecoveryStand
        jump_frames_++;
        if (jump_frames_ == 1) {
            fprintf(stderr, "\033[1;32m[Stage7] ✓ 跳跃成功，保持站立...\033[0m\n");
        }
        if (jump_frames_ >= 200) {  // 保持2秒后退出
            motion_.stop();
            done_ = true;
            fprintf(stderr, "\033[1;32m[Stage7] ✓ Stage7完成！\033[0m\n");
        }
        break;
    }

    default:
        break;
    }
}

bool Stage7::is_done() {
    return done_;
}
