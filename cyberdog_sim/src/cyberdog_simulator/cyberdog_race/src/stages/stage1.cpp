#include "cyberdog_race/stages/stage1.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// PD控制参数
static constexpr float KP_YAW        = 0.6f;
static constexpr float KD_YAW        = 0.08f;
static constexpr float BASE_SPEED    = 0.5f;
static constexpr float TURN_SPEED    = 0.25f;
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
    last_odom_x_    = sensor_.odom_x;
    last_odom_y_    = sensor_.odom_y;
    stuck_frames_   = 0;
    escape_frames_  = 0;
    lane_lost_frames_ = 0;
    motion_.locomotion();
    motion_.set_pitch(-0.26f);  // 低头约15度看地面黄线

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
        constexpr float TARGET_YAW = M_PI / 2.0f;
        float yaw_err = TARGET_YAW - sensor_.yaw;
        while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

        if (std::abs(yaw_err) < 0.05f) {
            done_ = true;
            motion_.set_pitch(0.0f);
            motion_.locomotion();  // 切回trot步态
            motion_.stop();
            #ifdef DEBUG_STAGE
            RCLCPP_INFO(rclcpp::get_logger("stage1"), "Stage1 done, yaw=%.3f", sensor_.yaw);
            fprintf(stderr, "\033[1;34m[Stage1] ✓ 转向完成 yaw=%.3f，第一赛段结束\033[0m\n", sensor_.yaw);
            #endif
        } else {
            // yaw_err>0 需要左转，发正yaw
            float turn = std::max(0.1f, std::min(0.5f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.0f, 0.0f, yaw_err > 0 ? turn : -turn);
            #ifdef DEBUG_MOTION
            RCLCPP_INFO(rclcpp::get_logger("stage1"),
                        "Turning: yaw=%.3f err=%.3f", sensor_.yaw, yaw_err);
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

    // 卡住检测：连续多帧位移过小视为被绊住，直接前进脱困
    float dx = sensor_.odom_x - last_odom_x_;
    float dy = sensor_.odom_y - last_odom_y_;
    float moved = std::sqrt(dx*dx + dy*dy);
    last_odom_x_ = sensor_.odom_x;
    last_odom_y_ = sensor_.odom_y;

    if (escape_frames_ > 0) {
        // 脱困模式：加速冲过石板
        motion_.set_velocity(0.5f, 0.f, 0.f);
        escape_frames_--;
        return;
    }

    if (moved < STUCK_DIST) {
        stuck_frames_++;
    } else {
        stuck_frames_ = 0;
    }

    if (stuck_frames_ >= STUCK_THRESH) {
        stuck_frames_ = 0;
        escape_frames_ = ESCAPE_FRAMES;
        #ifdef DEBUG_STAGE
        RCLCPP_WARN(rclcpp::get_logger("stage1"), "Stuck detected! Escaping...");
        #endif
        return;
    }

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

    // 直道：视觉+IMU 加权融合；弯道：纯视觉PD
    float d_offset = sensor_.lane_offset - prev_offset_;
    prev_offset_   = sensor_.lane_offset;
    float yaw_cmd  = 0.f;

    if (sensor_.lane_curvature < CURVE_THRESH) {
        // IMU 回正分量
        float yaw_err = -sensor_.yaw;
        while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;
        float imu_cmd = std::max(-0.4f, std::min(0.4f, yaw_err * 0.8f));

        // 视觉分量：双边权重高，单边权重低
        float vis_weight = sensor_.lane_both_sides ? 0.5f : 0.2f;
        float vis_cmd    = -(KP_YAW * sensor_.lane_offset + KD_YAW * d_offset);

        yaw_cmd = imu_cmd * (1.0f - vis_weight) + vis_cmd * vis_weight;
    } else {
        // 弯道：纯视觉PD
        float kp = sensor_.lane_both_sides ? KP_YAW : KP_YAW * 1.5f;
        float kd = sensor_.lane_both_sides ? KD_YAW : KD_YAW * 1.2f;
        yaw_cmd = -(kp * sensor_.lane_offset + kd * d_offset);
    }

    // 低通滤波，减少抖动时振荡
    static float yaw_cmd_filtered = 0.f;
    yaw_cmd_filtered = 0.6f * yaw_cmd_filtered + 0.4f * yaw_cmd;
    yaw_cmd = std::max(-0.6f, std::min(0.6f, yaw_cmd_filtered));

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
