#include "cyberdog_race/stages/stage6.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

static constexpr float PUSH_SPEED   = 0.1f;
static constexpr float KICK_SPEED   = 0.5f;
static constexpr float EXIT_DIST    = 3.0f; // Lidar判断出口的空旷距离阈值

void Stage6::init() {
    done_ = false;
    exit_angle_ = 0.0f;
    state_ = State::FIND_EXIT;
    motion_.stand();
#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage6"), "Stage6 init");
#endif
}

void Stage6::run() {
    if (done_) return;

    switch (state_) {
        case State::FIND_EXIT:
            if (sensor_.lidar_front > EXIT_DIST) {
                exit_angle_ = 0.0f;
                state_ = State::FIND_BALL;
#ifdef DEBUG_STAGE
                RCLCPP_INFO(rclcpp::get_logger("stage6"), "Exit found, lidar_front=%.2f", sensor_.lidar_front);
#endif
            } else {
                motion_.set_velocity(0.0f, 0.0f, 0.3f);
#ifdef DEBUG_MOTION
                RCLCPP_INFO(rclcpp::get_logger("stage6"), "Scanning for exit, lidar_front=%.2f", sensor_.lidar_front);
#endif
            }
            break;

        case State::FIND_BALL:
            if (sensor_.ball_found) {
                state_ = State::PUSH_BALL;
                motion_.stop();
#ifdef DEBUG_STAGE
                RCLCPP_INFO(rclcpp::get_logger("stage6"), "Ball found, x=%.2f dist=%.2f", sensor_.ball_x, sensor_.ball_dist);
#endif
            } else {
                motion_.set_velocity(0.0f, 0.0f, 0.2f);
#ifdef DEBUG_MOTION
                RCLCPP_INFO(rclcpp::get_logger("stage6"), "Searching ball...");
#endif
            }
            break;

        case State::PUSH_BALL: {
            float yaw = -0.5f * sensor_.ball_x;
            if (sensor_.ball_dist > 0.3f) {
                motion_.set_velocity(PUSH_SPEED, 0.0f, yaw);
#ifdef DEBUG_MOTION
                RCLCPP_INFO(rclcpp::get_logger("stage6"), "Pushing: dist=%.2f yaw=%.2f", sensor_.ball_dist, yaw);
#endif
            } else {
                state_ = State::KICK;
#ifdef DEBUG_STAGE
                RCLCPP_INFO(rclcpp::get_logger("stage6"), "Ball at goal, kicking");
#endif
            }
            break;
        }

        case State::KICK:
            motion_.set_velocity(KICK_SPEED, 0.0f, 0.0f);
            state_ = State::TO_FINISH;
#ifdef DEBUG_STAGE
            RCLCPP_INFO(rclcpp::get_logger("stage6"), "Kick!");
#endif
            break;

        case State::TO_FINISH:
            state_ = State::LIE_DOWN;
            break;

        case State::LIE_DOWN:
            motion_.lie_down();
            done_ = true;
#ifdef DEBUG_STAGE
            RCLCPP_INFO(rclcpp::get_logger("stage6"), "Stage6 done, lying down");
#endif
            break;
    }
}

bool Stage6::is_done() { return done_; }
