#include "cyberdog_race/stages/virtual/stage2.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

#ifdef DEBUG_STAGE
#define LOG_GREEN(msg) LOG_STAGE_GREEN("Stage2", msg)
#define LOG_GREENF(fmt, ...) LOG_STAGE_GREENF("Stage2", fmt, ##__VA_ARGS__)
#else
#define LOG_GREEN(msg)
#define LOG_GREENF(fmt, ...)
#endif

void Stage2::init() {
    done_      = false;
    wp_idx_    = 0;
    state_     = State::MOVE_TO_POINT;
    scan_found_   = false;
    scan_done_    = false;
    scan_confirm_ = 0;
    scan_wait_    = 0;
    hit_started_  = false;
    exit_started_ = false;
    exit_turning_ = false;

     waypoints_[0]  = {0.5f,  1.0f,   M_PI/2.f,     true};   // 第1行橙球前方
    waypoints_[1]  = {1.4f,  1.7f,   M_PI/2.f,     false};  // 第1-2行间过渡
    waypoints_[2]  = {2.5f,  1.7f,   M_PI/2.f,     true};   // 第2行橙球前方
    waypoints_[3]  = {2.9f,  2.6f,   M_PI/2.f,     true};   // 第3行橙球前方
    waypoints_[4]  = {2.9f,  3.5f,   M_PI/2.f,     false};  // 第3-4行间过渡
    waypoints_[5]  = {-0.1f, 3.5f,   M_PI/2.f,     true};   // 第4行橙球前方
    waypoints_[6]  = {-0.4f, 4.2f,   M_PI/2.f,     false};  // 出口


    target_x_   = sensor_.odom_x;
    target_y_   = 1.0f;
    target_yaw_ = M_PI;

    motion_.locomotion();

#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage2"),
                "Stage2 init, entry x=%.2f yaw=%.3f",
                sensor_.odom_x, sensor_.yaw);
#endif
}

void Stage2::run() {
    if (done_) return;

    switch (state_) {

    case State::MOVE_TO_POINT:
        if (reached_pos(target_x_, target_y_)) {
            state_ = State::TURN_TO_YAW;
            LOG_GREENF("✓ 到达位置 (%.2f, %.2f)", target_x_, target_y_);
        } else {
            if (wp_idx_ == 0 && std::abs(sensor_.odom_y - target_y_) < POS_THRESH) {
                state_ = State::TURN_TO_YAW;
                LOG_GREENF("✓ 到达 y=%.2f（忽略x）", target_y_);
            } else {
                navigate_to(target_x_, target_y_);
            }
        }
        break;

    case State::TURN_TO_YAW:
        if (reached_yaw(target_yaw_)) {
            int scan_dir = 0;

            // 写死扫描方向：
            // WP[0]：只右扫
            // WP[2]：只左扫
            // WP[3]：只右扫
            // WP[5]：只左扫
            if (wp_idx_ > 0 && wp_idx_ <= NUM_WP && waypoints_[wp_idx_ - 1].scan) {
                int current_wp = wp_idx_ - 1;

                if (current_wp == 0) {
                    scan_dir = -1;   // 右扫
                } else if (current_wp == 2) {
                    scan_dir = 1;    // 左扫
                } else if (current_wp == 3) {
                    scan_dir = -1;   // 右扫
                } else if (current_wp == 5) {
                    scan_dir = 1;    // 左扫
                }
            }

            bool do_scan = (scan_dir != 0) && !scan_done_;

            if (do_scan) {
                scan_start_yaw_ = sensor_.yaw;
                scan_found_     = false;
                scan_done_      = false;
                scan_confirm_   = 0;
                scan_wait_      = 0;

                if (scan_dir == 1) {
                    state_ = State::SCAN_LEFT;
                    LOG_GREENF("✓ 转向完成 yaw=%.2f，只左扫", sensor_.yaw);
                } else {
                    state_ = State::SCAN_RIGHT;
                    LOG_GREENF("✓ 转向完成 yaw=%.2f，只右扫", sensor_.yaw);
                }
            } else {
                LOG_GREENF("✓ 转向完成 yaw=%.2f", sensor_.yaw);
                scan_done_ = false;
                next_waypoint();
            }
        } else {
            turn_to(target_yaw_);
        }
        break;

    case State::SCAN_LEFT: {
        float yaw_diff = norm_yaw(sensor_.yaw - scan_start_yaw_);

        if (sensor_.ball_found && sensor_.ball_dist < BALL_DIST_THRESH) {
            scan_confirm_++;
        } else {
            scan_confirm_ = 0;
        }

        if (yaw_diff < SCAN_ANGLE) {
            motion_.set_velocity(0.f, 0.f, SCAN_TURN_SPEED);
        } else {
            motion_.stop();
            scan_wait_ = 0;

            if (scan_confirm_ > 0) {
                scan_found_ = true;
                state_   = State::HIT_BALL;
                LOG_GREENF("✓ 左扫转动中发现橙球！dist=%.2f", sensor_.ball_dist);
            } else {
                state_ = State::SCAN_LEFT_CHECK;
                LOG_GREENF("→ 左扫到位 yaw=%.2f，停下识别...", sensor_.yaw);
            }
        }
        break;
    }

    case State::SCAN_LEFT_CHECK: {
        scan_wait_++;

        if (sensor_.ball_found && sensor_.ball_dist < BALL_DIST_THRESH) {
            scan_wait_  = 0;
            scan_found_ = true;
            state_   = State::HIT_BALL;
            LOG_GREENF("✓ 左扫发现橙球！dist=%.2f", sensor_.ball_dist);
        } else if (scan_wait_ >= SCAN_WAIT_FRAMES) {
            scan_wait_ = 0;
            state_ = State::SCAN_LEFT_RETURN;
            LOG_GREEN("→ 左扫未发现球，转回后直接去下一点");
        }
        break;
    }

    case State::SCAN_LEFT_RETURN: {
        float err = norm_yaw(scan_start_yaw_ - sensor_.yaw);

        if (std::abs(err) < YAW_THRESH) {
            motion_.stop();
            LOG_GREEN("✓ 左扫结束，直接前往下一点");
            scan_done_ = true;
            next_waypoint();
        } else {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, err > 0 ? cmd : -cmd);
        }
        break;
    }

    case State::SCAN_RIGHT: {
        float yaw_diff = norm_yaw(sensor_.yaw - scan_start_yaw_);

        if (sensor_.ball_found && sensor_.ball_dist < BALL_DIST_THRESH) {
            scan_confirm_++;
        } else {
            scan_confirm_ = 0;
        }

        if (yaw_diff > -SCAN_ANGLE) {
            motion_.set_velocity(0.f, 0.f, -SCAN_TURN_SPEED);
        } else {
            motion_.stop();
            scan_wait_ = 0;

            if (scan_confirm_ > 0) {
                scan_found_ = true;
                state_   = State::HIT_BALL;
                LOG_GREENF("✓ 右扫转动中发现橙球！dist=%.2f", sensor_.ball_dist);
            } else {
                state_ = State::SCAN_RIGHT_CHECK;
                LOG_GREENF("→ 右扫到位 yaw=%.2f，停下识别...", sensor_.yaw);
            }
        }
        break;
    }

    case State::SCAN_RIGHT_CHECK: {
        scan_wait_++;

        if (sensor_.ball_found && sensor_.ball_dist < BALL_DIST_THRESH) {
            scan_wait_  = 0;
            scan_found_ = true;
            state_   = State::HIT_BALL;
            LOG_GREENF("✓ 右扫发现橙球！dist=%.2f", sensor_.ball_dist);
        } else if (scan_wait_ >= SCAN_WAIT_FRAMES) {
            scan_wait_ = 0;
            state_ = State::SCAN_RIGHT_RETURN;
            LOG_GREEN("→ 右扫未发现球，转回后直接去下一点");
        }
        break;
    }

    case State::SCAN_RIGHT_RETURN: {
        float err = norm_yaw(scan_start_yaw_ - sensor_.yaw);

        if (std::abs(err) < YAW_THRESH) {
            motion_.stop();
            LOG_GREEN("✓ 右扫结束，直接前往下一点");
            scan_done_ = true;
            next_waypoint();
        } else {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, err > 0 ? cmd : -cmd);
        }
        break;
    }

    case State::HIT_BALL:
#ifdef DEBUG_NO_HIT
        motion_.stop();
        state_ = State::MOVE_TO_POINT;
#else
    {
        if (!hit_started_) {
            hit_start_x_ = sensor_.odom_x;
            hit_start_y_ = sensor_.odom_y;
            hit_started_ = true;
        }

        float dx = sensor_.odom_x - hit_start_x_;
        float dy = sensor_.odom_y - hit_start_y_;
        float moved = std::sqrt(dx * dx + dy * dy);

        if (moved < 0.3f) {
            float yaw_cmd = sensor_.ball_found ? (-0.5f * sensor_.ball_x) : 0.f;
            motion_.set_velocity(HIT_SPEED, 0.f, yaw_cmd);
        } else {
            hit_started_ = false;
            motion_.stop();
            LOG_GREENF("✓ 撞球完成，直接前往下一点");
            next_waypoint();
        }
    }
#endif
        break;

    case State::DONE:
    {
        if (!exit_started_) {
            exit_start_x_ = sensor_.odom_x;
            exit_start_y_ = sensor_.odom_y;
            exit_started_ = true;
            exit_turning_ = false;
        }

        float dx = sensor_.odom_x - exit_start_x_;
        float dy = sensor_.odom_y - exit_start_y_;
        float moved = std::sqrt(dx * dx + dy * dy);

        if (moved < 0.2f) {
            motion_.set_velocity(MOVE_SPEED, 0.f, 0.f);
        } else if (!exit_turning_) {
            exit_target_yaw_ = sensor_.yaw - 0.175f;
            exit_turning_ = true;
            motion_.stop();
        } else {
            float yaw_err = norm_yaw(exit_target_yaw_ - sensor_.yaw);

            if (std::abs(yaw_err) > 0.05f) {
                float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
                motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
            } else {
                exit_started_ = false;
                exit_turning_ = false;
                done_ = true;
                motion_.stop();
                LOG_GREEN("✓ 赛段2结束，前进+右转完成");
            }
        }
        break;
    }

    default:
        break;
    }
}

bool Stage2::is_done() {
    return done_;
}

void Stage2::navigate_to(float tx, float ty) {
    float dx = tx - sensor_.odom_x;
    float dy = ty - sensor_.odom_y;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (wp_idx_ == 0) {
        float speed = std::min(MOVE_SPEED, std::abs(dy));
        motion_.set_velocity(speed, 0.f, 0.f);
        return;
    }

    static const float BALL_X[4] = {-0.4f, 0.8f, 2.0f, 3.2f};
    static const float BALL_Y[4] = {1.3f, 2.1f, 2.9f, 3.7f};
    static constexpr float DANGER_R = 0.1f;

    float min_ball_dist = 99.f;

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            float bdx = sensor_.odom_x - BALL_X[c];
            float bdy = sensor_.odom_y - BALL_Y[r];
            float bd = std::sqrt(bdx * bdx + bdy * bdy);

            if (bd < min_ball_dist) {
                min_ball_dist = bd;
            }
        }
    }

    float speed_limit = MOVE_SPEED;

    if (min_ball_dist < DANGER_R) {
        speed_limit = MOVE_SPEED * (min_ball_dist / DANGER_R) * 0.5f;
        speed_limit = std::max(0.05f, speed_limit);

#ifdef DEBUG_MOTION
        RCLCPP_INFO(rclcpp::get_logger("stage2"),
                    "Near ball! dist=%.2f speed_limit=%.2f",
                    min_ball_dist, speed_limit);
#endif
    }

    float desired_yaw = std::atan2(dy, dx);
    float yaw_err = norm_yaw(desired_yaw - sensor_.yaw);

#ifdef DEBUG_MOTION
    RCLCPP_INFO(rclcpp::get_logger("stage2"),
                "nav: pos=(%.2f,%.2f) tgt=(%.2f,%.2f) des_yaw=%.2f cur_yaw=%.2f err=%.2f",
                sensor_.odom_x, sensor_.odom_y,
                tx, ty,
                desired_yaw, sensor_.yaw, yaw_err);
#endif

    if (std::abs(yaw_err) > 0.35f) {
        float cmd = std::max(0.05f, std::min(0.5f, std::abs(yaw_err) * 0.8f));
        motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
    } else {
        float speed = std::min(speed_limit, dist);
        float yaw_cmd = std::max(-0.5f, std::min(0.5f, yaw_err * 2.0f));
        motion_.set_velocity(speed, 0.f, yaw_cmd);
    }
}

void Stage2::turn_to(float target_yaw) {
    float err = norm_yaw(target_yaw - sensor_.yaw);
    float cmd = std::max(0.2f, std::min(0.8f, std::abs(err) * 1.2f));

#ifdef DEBUG_MOTION
    RCLCPP_INFO(rclcpp::get_logger("stage2"),
                "turn_to: cur=%.2f target=%.2f err=%.2f cmd=%.2f",
                sensor_.yaw, target_yaw, err, err > 0 ? cmd : -cmd);
#endif

    motion_.set_velocity(0.f, 0.f, err > 0 ? cmd : -cmd);
}

bool Stage2::reached_pos(float tx, float ty) {
    float dx = tx - sensor_.odom_x;
    float dy = ty - sensor_.odom_y;
    return std::sqrt(dx * dx + dy * dy) < POS_THRESH;
}

bool Stage2::reached_yaw(float target_yaw) {
    return std::abs(norm_yaw(target_yaw - sensor_.yaw)) < YAW_THRESH;
}

void Stage2::next_waypoint() {
    scan_done_ = false;

    if (wp_idx_ >= 7) {
        state_ = State::DONE;
        LOG_GREEN("✓ 所有路径点完成，赛段2结束");
        return;
    }

    auto& wp = waypoints_[wp_idx_++];

    target_x_   = wp.x;
    target_y_   = wp.y;
    target_yaw_ = wp.yaw;
    state_      = State::MOVE_TO_POINT;

    LOG_GREENF("→ 走向 WP[%d] (%.2f, %.2f)", wp_idx_ - 1, wp.x, wp.y);

#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage2"),
                "Next WP[%d] (%.2f,%.2f) yaw=%.2f scan=%d",
                wp_idx_ - 1, wp.x, wp.y, wp.yaw, wp.scan);
#endif
}

