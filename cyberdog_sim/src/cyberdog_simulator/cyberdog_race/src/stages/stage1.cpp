#include "cyberdog_race/stages/stage1.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// PD控制参数
static constexpr float KP_YAW        = 0.5f;
static constexpr float KD_YAW        = 0.08f;
static constexpr float BASE_SPEED    = 0.3f;
static constexpr float TURN_SPEED    = 0.15f;
static constexpr float CURVE_THRESH  = 60.0f;
static constexpr float TURN_DONE_YAW = 1.3f;

// 路口检测参数
static constexpr float JUNCTION_LIDAR_THRESH = 1.2f;  // 前方障碍距离小于此值认为到路口
static constexpr float JUNCTION_TURN_YAW     = 1.57f; // 路口原地转向角度（约90度）
static constexpr float JUNCTION_TURN_SPEED   = 0.3f;  // 原地转向角速度

void Stage1::init() {
    done_           = false;
    in_turn_        = false;
    at_junction_    = false;
    yaw_start_      = 0.0f;
    prev_offset_    = 0.0f;
    run_frames_     = 0;
    motion_.locomotion();
    motion_.set_pitch(0.4f);  // 低头看地面黄线

    #ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage1"), "Stage1 init");
    #endif
}

void Stage1::run() {
    if (done_) return;

    // 里程计+IMU 实时打印（每秒一次），用于标定关键位置坐标
    #ifdef DEBUG_SENSOR
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("stage1"),
        clock, 1000,
        "[ODOM] x=%.3f y=%.3f yaw=%.3f | lane_valid=%d off=%.2f lidar=%.2f",
        sensor_.odom_x, sensor_.odom_y, sensor_.yaw,
        sensor_.lane_valid, sensor_.lane_offset, sensor_.lidar_front);
    #endif

    // ── 路口原地转向阶段 ──────────────────────────────────────
    if (at_junction_) {
        // 目标：转到世界坐标系 y 轴正方向（yaw = π/2）
        constexpr float TARGET_YAW = -M_PI / 2.0f;
        float yaw_err = TARGET_YAW - sensor_.yaw;
        // 归一化到 [-π, π]
        while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

        if (std::abs(yaw_err) < 0.05f) {
            done_ = true;
            motion_.set_pitch(0.0f);  // 抬头恢复正常
            motion_.stop();
            #ifdef DEBUG_STAGE
            RCLCPP_INFO(rclcpp::get_logger("stage1"),
                        "Stage1 done (junction turn complete), yaw=%.3f target=%.3f", sensor_.yaw, TARGET_YAW);
            #endif
        } else {
            // P控制转向，限制在 [0.08, 0.4]，误差小时慢转避免超调
            float turn = std::max(0.05f, std::min(0.3f, std::abs(yaw_err) * 0.4f));
            motion_.set_velocity(0.0f, 0.0f, (yaw_err > 0 ? turn : -turn));
            #ifdef DEBUG_MOTION
            RCLCPP_INFO(rclcpp::get_logger("stage1"),
                        "Turning: yaw=%.3f err=%.3f cmd=%.2f", sensor_.yaw, yaw_err, (yaw_err > 0 ? turn : -turn));
            #endif
        }
        return;
    }

    // ── 路口检测（里程计 x 坐标触发）────────────────────────
    if (sensor_.odom_x >= 2.95f && sensor_.odom_x <= 3.2f) {
        at_junction_ = true;
        yaw_start_   = sensor_.yaw;
        motion_.stop();
        #ifdef DEBUG_STAGE
        RCLCPP_INFO(rclcpp::get_logger("stage1"),
                    "Junction detected by odom x=%.2f, starting turn",
                    sensor_.odom_x);
        #endif
        return;
    }

    // ── 正常巡线阶段 ──────────────────────────────────────────
    if (!sensor_.lane_valid) {
        lane_lost_frames_++;
        if (lane_lost_frames_ > 10) {
            motion_.stop();
            #ifdef DEBUG_STAGE
            RCLCPP_WARN(rclcpp::get_logger("stage1"), "Lane lost, stopping");
            #endif
        }
        return;
    }
    lane_lost_frames_ = 0;

    // PD控制
    float d_offset = sensor_.lane_offset - prev_offset_;
    float yaw_cmd  = -(KP_YAW * sensor_.lane_offset + KD_YAW * d_offset);
    prev_offset_   = sensor_.lane_offset;

    float speed = BASE_SPEED;
    if (sensor_.lane_curvature > CURVE_THRESH) {
        speed = TURN_SPEED;
    }

    motion_.set_velocity(speed, 0.0f, yaw_cmd);

    #ifdef DEBUG_MOTION
    RCLCPP_INFO(rclcpp::get_logger("stage1"),
                "spd=%.2f yaw=%.2f off=%.2f curv=%.1f lidar=%.2f",
                speed, yaw_cmd, sensor_.lane_offset,
                sensor_.lane_curvature, sensor_.lidar_front);
    #endif
}

bool Stage1::is_done() { return done_; }
