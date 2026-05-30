#include "cyberdog_race/stages/stage6.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

void Stage6::init() {
    done_       = false;
    state_      = State::STAND;
#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage6"), "Stage6 init, odom=(%.2f,%.2f)",
                sensor_.odom_x, sensor_.odom_y);
#endif
}

void Stage6::run() {
    if (done_) return;

#ifdef DEBUG_SENSOR
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("stage6"), clock, 1000,
        "[ODOM] x=%.3f y=%.3f yaw=%.3f | white=%d wx=%.2f wd=%.2f",
        sensor_.odom_x, sensor_.odom_y, sensor_.yaw, sensor_.white_ball_found, 
        sensor_.white_ball_x, sensor_.white_ball_dist);
#endif

    switch (state_) {

    // ── ① 退出跳跃模式 → 直接转向 ──
    case State::STAND: {
        // 用gamepad b按钮=kRecoveryStand (12)退出kJump3d，不用LCM
        // kJump3d→kRecoveryStand是合法转换，LCM→gamepad反正收不到
        if (turn_frames_ == 0) {
            fprintf(stderr, "\033[1;34m[Stage6] 发送RecoveryStand退出跳跃...\033[0m\n");
        }
        motion_.recovery();  // gamepad b=1 → kRecoveryStand
        turn_frames_++;
        if (turn_frames_ > 300) {  // 等3秒完成恢复
            motion_.locomotion();
            state_ = State::TURN_X_NEG_INIT;
            turn_frames_ = 0;
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 恢复完成，开始转向\033[0m\n");
        }
        break;
    }

    // ── ② 原地转向x负方向 ──
    case State::TURN_X_NEG_INIT: {
        float yaw_err = norm_yaw(M_PI - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.3f, std::min(0.6f, std::abs(yaw_err) * 0.8f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
            state_ = State::GO_TO_WP1;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 转向x负完成，走向(%.2f,%.2f)\033[0m\n", 
                    WP1_X, WP1_Y);
#endif
        }
        break;
    }

    // ── ③ 走到 (2.35, 13.5) ──
    case State::GO_TO_WP1: {
        float dx = WP1_X - sensor_.odom_x;
        float dy = WP1_Y - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.2f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            if (std::abs(yaw_err) > 0.052f) {
                float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
                motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
            } else {
                float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
                motion_.set_velocity(SPEED, 0.f, yaw_cmd);
            }
        } else {
            motion_.stop();
            state_ = State::TURN_Y_POS;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 到达(%.2f,%.2f)，转向y正方向\033[0m\n", 
                    WP1_X, WP1_Y);
#endif
        }
        break;
    }

    // ── ③ 原地转向y正方向 ──
    case State::TURN_Y_POS: {
        float yaw_err = norm_yaw(M_PI_2 - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.3f, std::min(0.6f, std::abs(yaw_err) * 0.8f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
            state_ = State::GO_TO_WP2;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 转向y正完成，走向(%.2f,%.2f)\033[0m\n", 
                    WP2_X, WP2_Y);
#endif
        }
        break;
    }

    // ── ④ y正向走到 (2.35, 14.75) ──
    case State::GO_TO_WP2: {
        float dx = WP2_X - sensor_.odom_x;
        float dy = WP2_Y - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.2f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            if (std::abs(yaw_err) > 0.052f) {
                float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
                motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
            } else {
                float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
                motion_.set_velocity(SPEED, 0.f, yaw_cmd);
            }
        } else {
            motion_.stop();
            state_ = State::TURN_X_NEG;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 到达(%.2f,%.2f)，转向x负方向\033[0m\n", 
                    WP2_X, WP2_Y);
#endif
        }
        break;
    }

    // ── ⑤ 原地转向x负方向 ──
    case State::TURN_X_NEG: {
        float yaw_err = norm_yaw(M_PI - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.3f, std::min(0.6f, std::abs(yaw_err) * 0.8f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
            state_ = State::GO_TO_WP3;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 转向x负完成，走向(%.2f,%.2f)\033[0m\n", 
                    WP3_X, WP3_Y);
#endif
        }
        break;
    }

    // ── ⑥ x负向走到 (0.25, 14.5) ──
    case State::GO_TO_WP3: {
        float dx = WP3_X - sensor_.odom_x;
        float dy = WP3_Y - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.2f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            if (std::abs(yaw_err) > 0.052f) {
                float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
                motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
            } else {
                float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
                motion_.set_velocity(SPEED, 0.f, yaw_cmd);
            }
        } else {
            motion_.stop();
            state_ = State::TURN_Y_NEG;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 到达(%.2f,%.2f)，转向y负方向\033[0m\n", 
                    WP3_X, WP3_Y);
#endif
        }
        break;
    }

    // ── ⑦ 原地转向西偏南10° ──
    case State::TURN_Y_NEG: {
        float yaw_err = norm_yaw(M_PI - 0.1745f - sensor_.yaw);  // 西偏南10°
        if (std::abs(yaw_err) > 0.05f) {
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? 1.0f : -1.0f);
        } else {
            motion_.stop();
            state_ = State::GO_TO_WP4;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 转向西完成\033[0m\n");
#endif
        }
        break;
    }

    // ── ⑩ 走到 (0.15, 13.0) ──
    case State::GO_TO_WP4: {
        float dx = WP4_X - sensor_.odom_x;
        float dy = WP4_Y - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.15f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            if (std::abs(yaw_err) > 0.052f) {
                float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
                motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
            } else {
                float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
                motion_.set_velocity(SPEED, 0.f, yaw_cmd);
            }
        } else {
            motion_.stop();
            state_ = State::TURN_X_POS_SLOW;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 到达(%.2f,%.2f)，慢速转向x正\033[0m\n", WP4_X, WP4_Y);
#endif
        }
        break;
    }

    // ── ⑪ 慢速转向x正方向（0.25 rad/s） ──
    case State::TURN_X_POS_SLOW: {
        float yaw_err = norm_yaw(0.0f - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.08f, std::min(1.0f, std::abs(yaw_err) * 0.4f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
            state_ = State::GO_TO_WP5;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 转向x正完成，走到(%.2f,%.2f)\033[0m\n", WP5_X, WP5_Y);
#endif
        }
        break;
    }

    // ── ⑫ 走到 (2.35, 13.0) ──
    case State::GO_TO_WP5: {
        float dx = WP5_X - sensor_.odom_x;
        float dy = WP5_Y - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.15f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            if (std::abs(yaw_err) > 0.052f) {
                float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
                motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
            } else {
                float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
                motion_.set_velocity(SPEED, 0.f, yaw_cmd);
            }
        } else {
            motion_.stop();
            state_ = State::GO_TO_WP6;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 到达(%.2f,%.2f)，走向(%.1f,%.1f)\033[0m\n", WP5_X, WP5_Y, WP6_X, WP6_Y);
#endif
        }
        break;
    }

    // ── ⑬ 走到 (3.1, 13.0) ──
    case State::GO_TO_WP6: {
        float dx = WP6_X - sensor_.odom_x;
        float dy = WP6_Y - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.15f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            if (std::abs(yaw_err) > 0.052f) {
                float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
                motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
            } else {
                float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
                motion_.set_velocity(SPEED, 0.f, yaw_cmd);
            }
        } else {
            motion_.stop();
            state_ = State::FINAL_LIE_DOWN;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage6] ✓ 到达(%.2f,%.2f)，趴下\033[0m\n", WP6_X, WP6_Y);
#endif
        }
        break;
    }

    // ── ⑭ 最后趴下 ──
    case State::FINAL_LIE_DOWN: {
        motion_.lie_down();
        done_ = true;
#ifdef DEBUG_STAGE
        fprintf(stderr, "\033[1;32m[Stage6] ✓ Stage6完成！\033[0m\n");
#endif
        break;
    }

    default:
        break;
    }
}

bool Stage6::is_done() {
    return done_;
}
