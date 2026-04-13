#include "cyberdog_race/stages/stage5.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>

static constexpr float BRIDGE_SPEED  = 0.15f;
static constexpr float KP_LATERAL    = 0.4f;
static constexpr float BRIDGE_LENGTH = 3.0f; // 估算独木桥长度（米）

void Stage5::init() {
    done_ = false;
    crossed_line_ = false;
    state_ = State::ON_BRIDGE;
    motion_.locomotion();
#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage5"), "Stage5 init");
#endif
}

void Stage5::run() {
    if (done_) return;

    switch (state_) {
        case State::ON_BRIDGE: {
            float yaw = 0.0f;
            if (sensor_.lane_valid) {
                yaw = -KP_LATERAL * sensor_.lane_offset;
            }
            if (sensor_.lidar_front < 0.1f) {
                motion_.stop();
#ifdef DEBUG_STAGE
                RCLCPP_WARN(rclcpp::get_logger("stage5"), "Bridge edge detected, stopping");
#endif
                return;
            }
            motion_.set_velocity(BRIDGE_SPEED, 0.0f, yaw);
#ifdef DEBUG_MOTION
            RCLCPP_INFO(rclcpp::get_logger("stage5"), "On bridge: odom_x=%.2f yaw=%.2f lidar=%.2f",
                        sensor_.odom_x, yaw, sensor_.lidar_front);
#endif
            if (sensor_.odom_x > BRIDGE_LENGTH) {
                state_ = State::WAIT_CROSS;
                motion_.stop();
#ifdef DEBUG_STAGE
                RCLCPP_INFO(rclcpp::get_logger("stage5"), "Bridge crossed, waiting");
#endif
            }
            break;
        }

        case State::WAIT_CROSS:
            // 确认四足越线后跳下
            // TODO: 用足底接触传感器确认
            crossed_line_ = true;
            if (crossed_line_) {
                state_ = State::JUMP;
            }
            break;

        case State::JUMP:
            // TODO: 发跳跃指令
            // motion_.jump_forward();
            done_ = true;
            break;
    }
}

bool Stage5::is_done() { return done_; }
