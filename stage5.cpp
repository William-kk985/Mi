#include "cyberdog_race/stages/stage5.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

constexpr float BRIDGE_SPEED_FLAT  = 0.4f;
constexpr float BRIDGE_SPEED_SLOPE = 0.4f;

constexpr float LATERAL_COMPENSATION = 0.23f;
constexpr float LATERAL_COMPENSATION_STRONG = 0.18f;

constexpr float WP1_Y = 12.35f;
constexpr float WP2_X = -0.6f;
constexpr float WP3_Y = 15.40f;
constexpr float WP3_THRESH = 0.12f;
constexpr float WP4_X = 3.35f;
constexpr float WP_THRESH = 0.3f;
constexpr float WP5_TURN_Y = 13.80f;

constexpr float N2_TARGET_X = -0.35f;
constexpr float E_TARGET_Y = 15.40f;

// 最后下桥参数
constexpr float DROP_TURN_YAW = -M_PI;
constexpr float DROP_SPEED_SLOW = 0.12f;
constexpr float DROP_SPEED_NORMAL = 0.20f;
constexpr int DROP_FINISH_FRAMES = 260;
constexpr int DROP_SETTLE_FRAMES = 25;
constexpr int DROP_FRONT_LEG_FRAMES = 90;
constexpr int DROP_PAUSE_FRAMES = 35;

void Stage5::init() {
    done_ = false;
    state_ = State::MOVE_NORTH_1;
    turn_frames_ = 0;
    jump_frames_ = 0;

    motion_.locomotion();
    motion_.set_pitch(0.0f);

#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage5"), "Stage5 init, odom=(%.2f,%.2f)",
                sensor_.odom_x, sensor_.odom_y);
#endif
}

void Stage5::run() {
    if (done_) return;

#ifdef DEBUG_SENSOR
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("stage5"), clock, 1000,
        "[ODOM] x=%.3f y=%.3f yaw=%.3f | state=%d",
        sensor_.odom_x, sensor_.odom_y, sensor_.yaw, (int)state_);
#endif

    switch (state_) {

    case State::MOVE_NORTH_1: {
        float dy = WP1_Y - sensor_.odom_y;

        if (dy > WP_THRESH) {
            float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
            float yaw = std::max(-0.4f, std::min(0.4f, 0.8f * yaw_err));
            motion_.set_velocity(BRIDGE_SPEED_FLAT, 0.0f, yaw);
        } else {
            motion_.set_velocity(0.0f, 0.0f, 0.0f);
            state_ = State::TURN_LEFT_1;
            turn_frames_ = 0;
        }
        break;
    }

    case State::TURN_LEFT_1: {
        turn_frames_++;

        float target_yaw = M_PI;
        float yaw_err = norm_yaw(target_yaw - sensor_.yaw);

        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.15f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.0f, 0.0f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.set_velocity(0.0f, 0.0f, 0.0f);
            state_ = State::MOVE_WEST;
        }
        break;
    }

    case State::MOVE_WEST: {
        float dx = sensor_.odom_x - WP2_X;

        if (dx > WP_THRESH) {
            float yaw_err = norm_yaw(M_PI - sensor_.yaw);
            float yaw = std::max(-0.4f, std::min(0.4f, 0.8f * yaw_err));
            motion_.set_velocity(BRIDGE_SPEED_SLOPE, LATERAL_COMPENSATION, yaw);
        } else {
            motion_.set_velocity(0.0f, 0.0f, 0.0f);
            state_ = State::TURN_RIGHT_1;
            turn_frames_ = 0;
        }
        break;
    }

    case State::TURN_RIGHT_1: {
        turn_frames_++;

        float target_yaw = M_PI / 2.0f;
        float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
        float vy_turn = LATERAL_COMPENSATION_STRONG;

        if (std::abs(yaw_err) < 0.03f && turn_frames_ > 15) {
            motion_.set_velocity(0.0f, 0.0f, 0.0f);
            turn_frames_ = 0;
            state_ = State::MOVE_NORTH_2;

#ifdef DEBUG_STAGE
            fprintf(stderr, "\033[1;34m[Stage5] ✓ 右转完成，开始向北\033[0m\n");
#endif
        } else {
            float cmd = std::max(0.3f, std::min(0.6f, std::abs(yaw_err) * 1.0f));
            motion_.set_velocity(0.15f, vy_turn, yaw_err > 0 ? cmd : -cmd);
        }
        break;
    }

    case State::MOVE_NORTH_2: {
        float dy = WP3_Y - sensor_.odom_y;

        if (dy > WP3_THRESH) {
            float x_error = sensor_.odom_x - N2_TARGET_X;

            float vy_cmd = 0.0f;
            if (std::abs(x_error) > 0.03f) {
                vy_cmd = 0.75f * x_error;
                vy_cmd = std::max(-0.18f, std::min(0.32f, vy_cmd));
            }

            float yaw_err = norm_yaw(M_PI / 2.0f - sensor_.yaw);
            float yaw_cmd = std::max(-0.28f, std::min(0.28f, yaw_err * 0.65f));

            motion_.set_velocity(0.24f, vy_cmd, yaw_cmd);
        } else {
            motion_.set_velocity(0.0f, 0.0f, 0.0f);
            state_ = State::TURN_RIGHT_2;
            turn_frames_ = 0;
        }
        break;
    }

    case State::TURN_RIGHT_2: {
        turn_frames_++;

        float target_yaw = 0.0f;
        float yaw_err = norm_yaw(target_yaw - sensor_.yaw);

        if (turn_frames_ < 10) {
            motion_.set_velocity(0.0f, 0.0f, 0.0f);
            break;
        }

        if (std::abs(yaw_err) > 0.05f) {
            float yaw_cmd = std::max(-0.45f, std::min(0.45f, yaw_err * 0.75f));
            motion_.set_velocity(0.0f, 0.0f, yaw_cmd);
        } else {
            motion_.set_velocity(0.0f, 0.0f, 0.0f);
            state_ = State::MOVE_EAST;
            turn_frames_ = 0;
        }
        break;
    }

    case State::MOVE_EAST: {
        float dx = WP4_X - sensor_.odom_x;

        if (dx > WP_THRESH) {
            float y_error = E_TARGET_Y - sensor_.odom_y;

            float vy_cmd = 0.0f;
            if (std::abs(y_error) > 0.03f) {
                vy_cmd = 0.65f * y_error;
                vy_cmd = std::max(-0.22f, std::min(0.28f, vy_cmd));
            }

            float yaw_err = norm_yaw(0.0f - sensor_.yaw);
            float yaw_cmd = std::max(-0.28f, std::min(0.28f, yaw_err * 0.65f));

            motion_.set_velocity(0.24f, vy_cmd, yaw_cmd);
        } else {
            motion_.set_velocity(0.0f, 0.0f, 0.0f);
            state_ = State::TURN_RIGHT_3;
            turn_frames_ = 0;
        }
        break;
    }

    case State::TURN_RIGHT_3: {
        turn_frames_++;

        float target_yaw = -M_PI / 2.0f;
        float yaw_err = norm_yaw(target_yaw - sensor_.yaw);

        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.15f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.0f, 0.0f, yaw_err > 0 ? cmd : -cmd);
        } else {
            motion_.set_velocity(0.0f, 0.0f, 0.0f);
            state_ = State::MOVE_SOUTH;
        }
        break;
    }

    case State::MOVE_SOUTH: {
        bool at_turn_point = sensor_.odom_y <= WP5_TURN_Y;

        if (!at_turn_point) {
            float yaw_err = norm_yaw(-M_PI / 2.0f - sensor_.yaw);
            float yaw = std::max(-0.28f, std::min(0.28f, 0.65f * yaw_err));
            motion_.set_velocity(0.28f, 0.0f, yaw);
        } else {
            motion_.set_velocity(0.0f, 0.0f, 0.0f);
            state_ = State::JUMP_PREPARE;
            turn_frames_ = 0;
            jump_frames_ = 0;
        }
        break;
    }

    case State::JUMP_PREPARE: {
    turn_frames_++;

    // 不右转了，直接往前走
    motion_.set_pitch(0.0f);
    motion_.set_velocity(0.38f, 0.0f, 0.0f);

    if (turn_frames_ > 80) {
        state_ = State::JUMP;
        jump_frames_ = 0;
        fprintf(stderr, "\033[1;33m[Stage5] ENTER STRAIGHT WALK DOWN yaw=%.3f\033[0m\n", sensor_.yaw);
    }

    break;
}

case State::JUMP: {
    jump_frames_++;

    // yaw=-1.57 时 vx 正方向就是往南，继续走
    motion_.set_pitch(0.0f);
    motion_.set_velocity(0.45f, 0.0f, 0.0f);

    // 不按帧结束，按 y 坐标结束，确保真的走下去
    if (sensor_.odom_y < 12.80f) {
    motion_.set_velocity(0.0f, 0.0f, 0.0f);
    done_ = true;
}

    break;
}

    }
}

bool Stage5::is_done() {
    return done_;
}