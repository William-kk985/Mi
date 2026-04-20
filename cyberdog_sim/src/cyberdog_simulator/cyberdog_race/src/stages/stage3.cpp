#include "cyberdog_race/stages/stage3.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

constexpr Stage3::WP Stage3::WAYPOINTS[Stage3::NUM_WP];

void Stage3::init() {
    done_         = false;
    wp_idx_       = 0;
    turning_to_y_ = false;
    motion_.locomotion();
    motion_.set_pitch(-0.52f);

#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage3"), "Stage3 init, odom=(%.2f,%.2f)",
                sensor_.odom_x, sensor_.odom_y);
#endif
}

void Stage3::run() {
    if (done_) return;

#ifdef DEBUG_SENSOR
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("stage3"), clock, 1000,
        "[ODOM] x=%.3f y=%.3f yaw=%.3f | wp=%d off=%.2f",
        sensor_.odom_x, sensor_.odom_y, sensor_.yaw, wp_idx_, sensor_.lane_offset);
#endif

    // 所有路径点走完后，先转向y正方向
    if (wp_idx_ >= NUM_WP && !turning_to_y_) {
        float yaw_err = M_PI / 2.0f - sensor_.yaw;
        while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
            return;
        }
        turning_to_y_ = true;
    }

    // 出口检测：到达 (EXIT_X, EXIT_Y) 后转向x轴负方向再结束
    float exit_dx = EXIT_X - sensor_.odom_x;
    float exit_dy = EXIT_Y - sensor_.odom_y;
    if (std::sqrt(exit_dx*exit_dx + exit_dy*exit_dy) < EXIT_THRESH) {
        float yaw_err = M_PI - sensor_.yaw;
        while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;
        if (std::abs(yaw_err) > 0.05f) {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? cmd : -cmd);
        } else {
            done_ = true;
            motion_.set_pitch(0.0f);
            motion_.stop();
#ifdef DEBUG_STAGE
            RCLCPP_INFO(rclcpp::get_logger("stage3"), "Stage3 done, yaw=%.2f", sensor_.yaw);
#endif
        }
        return;
    }

    // 切换路径点
    if (wp_idx_ < NUM_WP) {
        float dx = WAYPOINTS[wp_idx_].x - sensor_.odom_x;
        float dy = WAYPOINTS[wp_idx_].y - sensor_.odom_y;
        if (std::sqrt(dx*dx + dy*dy) < WP_THRESH) {
            wp_idx_++;
#ifdef DEBUG_STAGE
            RCLCPP_INFO(rclcpp::get_logger("stage3"), "WP %d reached, next=%d", wp_idx_-1, wp_idx_);
#endif
        }
    }

    // 计算目标方向
    float target_yaw;
    if (wp_idx_ < NUM_WP) {
        float dx = WAYPOINTS[wp_idx_].x - sensor_.odom_x;
        float dy = WAYPOINTS[wp_idx_].y - sensor_.odom_y;
        target_yaw = std::atan2(dy, dx);
    } else {
        // 路径点走完，朝y正方向偏x正方向20度
        target_yaw = M_PI / 2.0f - 0.35f;
    }

    float yaw_err = target_yaw - sensor_.yaw;
    while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

    // 主控：路径点方向；辅助：视觉微调
    float yaw_cmd = KP_IMU * yaw_err;
    if (sensor_.lane_valid) {
        yaw_cmd += KP_VIS * sensor_.lane_offset;
    }
    yaw_cmd = std::max(-0.5f, std::min(0.5f, yaw_cmd));

    motion_.set_velocity(SPEED, 0.0f, yaw_cmd);

#ifdef DEBUG_MOTION
    RCLCPP_INFO(rclcpp::get_logger("stage3"),
                "wp=%d tgt=%.2f err=%.2f cmd=%.2f off=%.2f",
                wp_idx_, target_yaw, yaw_err, yaw_cmd, sensor_.lane_offset);
#endif
}

bool Stage3::is_done() { return done_; }
