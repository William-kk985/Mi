#include "cyberdog_race/stages/stage4.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

constexpr Stage4::WP Stage4::WAYPOINTS[Stage4::NUM_WP];

void Stage4::init() {
    done_          = false;
    wp_idx_        = 0;
    state_         = State::MOVE_TO_WP;
    crouch_frames_ = 0;
    detect_wait_   = 0;
    crouch_active  = false;
    crouch_start_x_ = 0.f;
    crouch_start_y_ = 0.f;
    motion_.locomotion();
    motion_.set_pitch(0.0f);

#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage4"), "Stage4 init, odom=(%.2f,%.2f)",
                sensor_.odom_x, sensor_.odom_y);
#endif
}

void Stage4::run() {
    if (done_) return;

#ifdef DEBUG_SENSOR
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("stage4"), clock, 1000,
        "[ODOM] x=%.3f y=%.3f yaw=%.3f | state=%d wp=%d",
        sensor_.odom_x, sensor_.odom_y, sensor_.yaw, (int)state_, wp_idx_);
#endif

    switch (state_) {

    case State::MOVE_TO_WP: {
        if (wp_idx_ >= NUM_WP) {
            // 到达(2.15,9)，先停稳（50帧），再转向y正方向
            if (crouch_frames_ < 50) {
                crouch_frames_++;
                motion_.stop();
                break;
            }
            float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
            if (std::abs(yaw_err) > 0.05f) {
                float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
                motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
            } else {
                crouch_frames_ = 0;
                state_ = State::CROUCH_PASS;
                crouch_active  = true;
#ifdef DEBUG_STAGE
                fprintf(stderr, "\033[1;34m[Stage4] ✓ 到达(2.15,9.75)转向完成，开始蹲下\033[0m\n");
#endif
            }
            break;
        }
        float dx = WAYPOINTS[wp_idx_].x - sensor_.odom_x;
        float dy = WAYPOINTS[wp_idx_].y - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist < WP_THRESH) {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 到达路径点 WP%d (%.2f,%.2f)\033[0m\n",
                    wp_idx_, WAYPOINTS[wp_idx_].x, WAYPOINTS[wp_idx_].y);
#endif
            wp_idx_++;
            if (wp_idx_ == 1) {
                state_ = State::TURN_TO_Y;
            }
            break;
        }
        float target_yaw = std::atan2(dy, dx);
        float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
        float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
        motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        break;
    }

    case State::TURN_TO_Y: {
        // 在WP1(2.25,7.35)处转向y正方向(yaw=π/2)
        float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ WP0转向完成，继续走向(2.15,9.75)\033[0m\n");
#endif
            state_ = State::MOVE_TO_WP;
        }
        break;
    }

    case State::CROUCH_PASS: {
        if (crouch_frames_ == 0) {
            crouch_start_x_ = sensor_.odom_x;
            crouch_start_y_ = sensor_.odom_y;
        }
        crouch_frames_++;

        motion_.stop();

        // 等身体高度降到目标值附近（CROUCH_HEIGHT + 0.02m）
        float h = sensor_.body_height;
        if (h < CROUCH_HEIGHT + 0.02f && crouch_frames_ > 30) {
            state_ = State::DETECT_FOOTBALL;
            detect_wait_ = 0;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 蹲下完成 height=%.3f，开始识别足球\033[0m\n", h);
#endif
        }
        break;
    }

    case State::DETECT_FOOTBALL: {
        detect_wait_++;
        motion_.stop();
        if (vision_result.football_found) {
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;32m[Stage4] ✓ 识别到足球！dist=%.2f cx=%.2f，前进1.5m\033[0m\n",
                    vision_result.football_dist, vision_result.football_cx);
#endif
            crouch_start_y_ = sensor_.odom_y;  // 记录前进起点
            state_ = State::ADVANCE;
        } else if (detect_wait_ > 300) {
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] 足球识别超时，done\033[0m\n");
#endif
            state_ = State::DONE_WAIT;
        }
        break;
    }

    case State::ADVANCE: {
        float moved = sensor_.odom_y - crouch_start_y_;
        if (moved < 1.1f) {
            float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
            float yaw_cmd = std::max(-0.3f, std::min(0.3f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED * 0.8f, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 前进1.1m完成，开始后退\033[0m\n");
#endif
            state_ = State::RETREAT;
        }
        break;
    }

    case State::RETREAT: {
        if (sensor_.odom_y > 9.7f + 0.1f) {
            motion_.set_velocity(-SPEED * 0.5f, 0.f, 0.f);
        } else {
            motion_.stop();
            crouch_active = false;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 后退完成(y=%.2f)，恢复身高\033[0m\n", sensor_.odom_y);
#endif
            state_ = State::RECOVER_HEIGHT;
            crouch_frames_ = 0;
        }
        break;
    }

    case State::RECOVER_HEIGHT: {
        motion_.stop();
        if (sensor_.body_height > 0.23f) {
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 身高恢复完成 height=%.3f\033[0m\n", sensor_.body_height);
#endif
            state_ = State::EXIT_TURN;
        }
        break;
    }

    case State::EXIT_TURN: {
        float yaw_err = norm_yaw(-M_PI / 2.0f - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 转向y负方向完成，走到y=9.0\033[0m\n");
#endif
            state_ = State::EXIT_MOVE;
        }
        break;
    }

    case State::EXIT_MOVE: {
        if (sensor_.odom_y > 9.0f + 0.1f) {
            float yaw_err = norm_yaw(-M_PI / 2.0f - sensor_.yaw);
            float yaw_cmd = std::max(-0.3f, std::min(0.3f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 到达y=9.0，转向x负方向\033[0m\n");
#endif
            state_ = State::EXIT_TURN2;
        }
        break;
    }

    case State::EXIT_TURN2: {
        float yaw_err = norm_yaw(M_PI - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 转向x负方向完成，走到x=1.75\033[0m\n");
#endif
            state_ = State::EXIT_MOVE2;
        }
        break;
    }

    case State::EXIT_MOVE2: {
        if (sensor_.odom_x > 1.75f + 0.1f) {
            float yaw_err = norm_yaw(M_PI - sensor_.yaw);
            float yaw_cmd = std::max(-0.3f, std::min(0.3f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 到达x=1.75，转向-x偏+y 20度\033[0m\n");
#endif
            state_ = State::EXIT_TURN3;
        }
        break;
    }

    case State::EXIT_TURN3: {
        float target_yaw = M_PI - M_PI / 6.0f;
        float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
            crouch_start_x_ = sensor_.odom_x;
            crouch_start_y_ = sensor_.odom_y;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 转向-x偏+y 30度完成，前进0.3m\033[0m\n");
#endif
            state_ = State::EXIT_MOVE3;
        }
        break;
    }

    case State::EXIT_MOVE3: {
        float dx = sensor_.odom_x - crouch_start_x_;
        float dy = sensor_.odom_y - crouch_start_y_;
        float moved = std::sqrt(dx*dx + dy*dy);
        if (moved < 0.3f) {
            float target_yaw = M_PI - M_PI / 6.0f;
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            float yaw_cmd = std::max(-0.3f, std::min(0.3f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 前进0.3m完成\033[0m\n");
#endif
            state_ = State::EXIT_TURN4;
        }
        break;
    }

    case State::EXIT_TURN4: {
        float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 转向y正方向完成，走向(0.9,9.75)\033[0m\n");
#endif
            state_ = State::EXIT_MOVE4;
        }
        break;
    }

    case State::EXIT_MOVE4: {
        float dx = 0.9f - sensor_.odom_x;
        float dy = 9.75f - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.2f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 到达(0.9,9.75)，开始识别蓝球\033[0m\n");
#endif
            detect_wait_ = 0;
            state_ = State::DETECT_BLUE;
        }
        break;
    }

    case State::DETECT_BLUE: {
        motion_.stop();
        if (detect_wait_ == 0) {
            detect_wait_ = 1;
            detect_start_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
        }
        auto elapsed = (rclcpp::Clock(RCL_STEADY_TIME).now() - detect_start_time_).seconds();
        if (elapsed < 0.3) break;
        if (sensor_.blue_ball_found) {
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;32m[Stage4] ✓ 识别到蓝球！dist=%.2f cx=%.2f，前进1m\033[0m\n",
                    sensor_.blue_ball_dist, sensor_.blue_ball_x);
#endif
            crouch_start_x_ = sensor_.odom_x;
            crouch_start_y_ = sensor_.odom_y;
            state_ = State::ADVANCE_BLUE;
        } else if (elapsed > 3.3) {
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] 蓝球识别超时，done\033[0m\n");
#endif
            state_ = State::DONE_WAIT;
        }
        break;
    }

    case State::ADVANCE_BLUE: {
        float dx = sensor_.odom_x - crouch_start_x_;
        float dy = sensor_.odom_y - crouch_start_y_;
        float moved = std::sqrt(dx*dx + dy*dy);
        if (moved < 1.0f) {
            float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
            float yaw_cmd = std::max(-0.3f, std::min(0.3f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED * 0.8f, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 蓝球前进1m完成，转向y负方向\033[0m\n");
#endif
            state_ = State::EXIT_TURN5;
        }
        break;
    }

    case State::EXIT_TURN5: {
        float yaw_err = norm_yaw(-M_PI / 2.0f - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 转向y负方向完成，走到(0.9,9.15)\033[0m\n");
#endif
            state_ = State::EXIT_MOVE5;
        }
        break;
    }

    case State::EXIT_MOVE5: {
        float dx = 0.9f - sensor_.odom_x;
        float dy = 9.15f - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.2f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 到达(0.9,9.15)\033[0m\n");
#endif
            state_ = State::EXIT_TURN6;
        }
        break;
    }

    case State::EXIT_TURN6: {
        // 转向-x偏-y 15度：yaw = -(π - π/12) = -11π/12
        float target_yaw = -(M_PI - M_PI / 12.0f);
        float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 转向-x偏-y 15度完成\033[0m\n");
#endif
            state_ = State::EXIT_MOVE6;
        }
        break;
    }

    case State::EXIT_MOVE6: {
        float dx = 0.0f - sensor_.odom_x;
        float dy = 9.0f - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.2f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 到达(0,9.0)\033[0m\n");
#endif
            state_ = State::EXIT_TURN7;
        }
        break;
    }

    case State::EXIT_TURN7: {
        float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
            crouch_active  = true;
            crouch_frames_ = 0;
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 转向y正方向完成，开始蹲下\033[0m\n");
#endif
            state_ = State::CROUCH_PASS2;
        }
        break;
    }

    case State::CROUCH_PASS2: {
        crouch_frames_++;
        motion_.stop();
        float h = sensor_.body_height;
        if (h < CROUCH_HEIGHT + 0.02f && crouch_frames_ > 30) {
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 蹲下完成 height=%.3f，前进1m\033[0m\n", h);
#endif
            crouch_start_x_ = sensor_.odom_x;
            crouch_start_y_ = sensor_.odom_y;
            state_ = State::ADVANCE2;
        }
        break;
    }

    case State::ADVANCE2: {
        float dx = sensor_.odom_x - crouch_start_x_;
        float dy = sensor_.odom_y - crouch_start_y_;
        float moved = std::sqrt(dx*dx + dy*dy);
        if (moved < 1.0f) {
            float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
            float yaw_cmd = std::max(-0.3f, std::min(0.3f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED * 0.5f, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 前进1m完成，开始起立\033[0m\n");
#endif
            crouch_active  = false;
            crouch_frames_ = 0;
            state_ = State::RECOVER_HEIGHT2;
        }
        break;
    }

    case State::RECOVER_HEIGHT2: {
        motion_.stop();
        if (sensor_.body_height > 0.23f) {
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 起立完成 height=%.3f\033[0m\n", sensor_.body_height);
#endif
            crouch_start_x_ = sensor_.odom_x;
            crouch_start_y_ = sensor_.odom_y;
            state_ = State::ADVANCE3;
        }
        break;
    }

    case State::ADVANCE3: {
        float dx = sensor_.odom_x - crouch_start_x_;
        float dy = sensor_.odom_y - crouch_start_y_;
        float moved = std::sqrt(dx*dx + dy*dy);
        if (moved < 0.5f) {
            float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
            float yaw_cmd = std::max(-0.3f, std::min(0.3f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 前进0.5m完成\033[0m\n");
#endif
            state_ = State::FINAL_TURN;
        }
        break;
    }

    case State::FINAL_TURN: {
        float yaw_err = norm_yaw(-M_PI / 2.0f - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 转向y负方向完成，前进0.5m\033[0m\n");
#endif
            crouch_start_x_ = sensor_.odom_x;
            crouch_start_y_ = sensor_.odom_y;
            state_ = State::FINAL_MOVE;
        }
        break;
    }

    case State::FINAL_MOVE: {
        float dx = sensor_.odom_x - crouch_start_x_;
        float dy = sensor_.odom_y - crouch_start_y_;
        float moved = std::sqrt(dx*dx + dy*dy);
        if (moved < 0.5f) {
            float yaw_err = norm_yaw(-M_PI / 2.0f - sensor_.yaw);
            float yaw_cmd = std::max(-0.3f, std::min(0.3f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 前进0.5m完成，开始蹲下\033[0m\n");
#endif
            crouch_active  = true;
            crouch_frames_ = 0;
            state_ = State::CROUCH_PASS3;
        }
        break;
    }

    case State::CROUCH_PASS3: {
        crouch_frames_++;
        motion_.stop();
        float h = sensor_.body_height;
        if (h < CROUCH_HEIGHT + 0.02f && crouch_frames_ > 30) {
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 蹲下完成 height=%.3f，前进1m\033[0m\n", h);
#endif
            crouch_start_x_ = sensor_.odom_x;
            crouch_start_y_ = sensor_.odom_y;
            state_ = State::ADVANCE4;
        }
        break;
    }

    case State::ADVANCE4: {
        float dx = sensor_.odom_x - crouch_start_x_;
        float dy = sensor_.odom_y - crouch_start_y_;
        float moved = std::sqrt(dx*dx + dy*dy);
        if (moved < 1.0f) {
            float yaw_err = norm_yaw(-M_PI / 2.0f - sensor_.yaw);
            float yaw_cmd = std::max(-0.3f, std::min(0.3f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED * 0.5f, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 前进1m完成，开始起立\033[0m\n");
#endif
            crouch_active = false;
            state_ = State::RECOVER_HEIGHT3;
        }
        break;
    }

    case State::RECOVER_HEIGHT3: {
        motion_.stop();
        if (sensor_.body_height > 0.23f) {
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 起立完成 height=%.3f，走到(0.5,7.35)\033[0m\n", sensor_.body_height);
#endif
            state_ = State::FINAL_MOVE2;
        }
        break;
    }

    case State::FINAL_MOVE2: {
        float dx = 0.5f - sensor_.odom_x;
        float dy = 7.35f - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.2f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 到达(0.5,7.35)\033[0m\n");
#endif
            state_ = State::FINAL_TURN2;
        }
        break;
    }

    case State::FINAL_TURN2: {
        float yaw_err = norm_yaw(0.f - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 转向x正方向完成\033[0m\n");
#endif
            state_ = State::FINAL_MOVE3;
        }
        break;
    }

    case State::FINAL_MOVE3: {
        float dx = 2.9f - sensor_.odom_x;
        float dy = 7.35f - sensor_.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 0.2f) {
            float target_yaw = std::atan2(dy, dx);
            float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
            float yaw_cmd = std::max(-0.5f, std::min(0.5f, KP_IMU * yaw_err));
            motion_.set_velocity(SPEED, 0.f, yaw_cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 到达(2.9,7.35)\033[0m\n");
#endif
            state_ = State::FINAL_TURN3;
        }
        break;
    }

    case State::FINAL_TURN3: {
        float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.stop();
#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage4] ✓ 转向y正方向完成\033[0m\n");
#endif
            state_ = State::DONE_WAIT;
        }
        break;
    }

    case State::DONE_WAIT:
        done_ = true;
        motion_.stop();
#ifdef DEBUG_STAGE
        fprintf(stderr, "\033[1;34m[Stage4] Stage4 done\033[0m\n");
#endif
        break;
    }
}

bool Stage4::is_done() { return done_; }
