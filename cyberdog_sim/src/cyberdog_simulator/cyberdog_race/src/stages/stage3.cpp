#include "cyberdog_race/stages/stage3.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>

static constexpr float KP_YAW      = 0.5f;
static constexpr float BASE_SPEED  = 0.35f;
static constexpr float CURVE_SPEED = 0.18f;
static constexpr float CURVE_THRESH = 80.0f;

void Stage3::init() {
    done_ = false;
    motion_.locomotion();
#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage3"), "Stage3 init");
#endif
}

void Stage3::run() {
    if (done_) return;

    if (!sensor_.lane_valid) {
        done_ = true;
        motion_.stop();
#ifdef DEBUG_STAGE
        RCLCPP_INFO(rclcpp::get_logger("stage3"), "Stage3 done (lane lost)");
#endif
        return;
    }

    float speed = (sensor_.lane_yaw > CURVE_THRESH) ? CURVE_SPEED : BASE_SPEED;
    float yaw   = -KP_YAW * sensor_.lane_offset;

    motion_.set_velocity(speed, 0.0f, yaw);
#ifdef DEBUG_MOTION
    RCLCPP_INFO(rclcpp::get_logger("stage3"), "spd=%.2f yaw=%.2f off=%.2f lane_yaw=%.1f",
                speed, yaw, sensor_.lane_offset, sensor_.lane_yaw);
#endif
}

bool Stage3::is_done() { return done_; }
