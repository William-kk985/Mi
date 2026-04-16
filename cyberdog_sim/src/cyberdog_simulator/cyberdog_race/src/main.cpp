#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cyberdog_msg/msg/yaml_param.hpp>
#include <cv_bridge/cv_bridge.h>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <csignal>
#include <cmath>

#include "simulator_lcmt.hpp"
#include "cyberdog_race/debug_config.hpp"
#include "cyberdog_race/motion_ctrl.hpp"
#include "cyberdog_race/vision/lane_detector.hpp"
#include "cyberdog_race/vision/ball_detector.hpp"
#include "cyberdog_race/stages/stage_base.hpp"
#include "cyberdog_race/stages/stage1.hpp"
#include "cyberdog_race/stages/stage2.hpp"
#include "cyberdog_race/stages/stage3.hpp"
#include "cyberdog_race/stages/stage4.hpp"
#include "cyberdog_race/stages/stage5.hpp"
#include "cyberdog_race/stages/stage6.hpp"

// 全局指针，供信号处理访问
static MotionCtrl* g_motion = nullptr;

static void on_shutdown(int) {
    if (g_motion) {
        g_motion->stop();
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        g_motion->lie_down();
        RCLCPP_WARN(rclcpp::get_logger("race"), "Shutdown: robot stopped and lying down.");
    }
    rclcpp::shutdown();
}

// ============================================================
// 调试模式说明（修改 debug_config.hpp 后重新 build）：
//
//   DEBUG_SINGLE_STAGE N  → 只跑第N赛段，赛段结束后停止，不切换
//   DEBUG_START_STAGE  N  → 从第N赛段开始，走完整状态机
//   两者都不定义          → 正式比赛模式，从第1赛段完整跑
//
//   DEBUG_VISION  → 视觉 imshow 可视化
//   DEBUG_MOTION  → 运动指令日志
//   DEBUG_SENSOR  → 传感器数据日志
//   DEBUG_STAGE   → 状态机切换日志
// ============================================================

class RaceController : public rclcpp::Node {
public:
    RaceController() : Node("race_controller") {
        // 确定起始赛段
#if defined(DEBUG_SINGLE_STAGE)
        cur_stage_ = DEBUG_SINGLE_STAGE - 1;
        single_stage_mode_ = true;
        RCLCPP_WARN(get_logger(), "[DEBUG] Single stage mode: only running stage %d", DEBUG_SINGLE_STAGE);
#elif defined(DEBUG_START_STAGE)
        cur_stage_ = DEBUG_START_STAGE - 1;
        RCLCPP_WARN(get_logger(), "[DEBUG] Starting from stage %d", DEBUG_START_STAGE);
#else
        cur_stage_ = 0;
#endif

        // 订阅传感器话题
        auto qos_be = rclcpp::QoS(10).best_effort();
        sub_rgb_ = create_subscription<sensor_msgs::msg::Image>(
            "/RGB_camera/image_raw", qos_be,
            [this](sensor_msgs::msg::Image::SharedPtr msg) { on_rgb(msg); });

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos_be,
            [this](sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });

        sub_lidar_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { on_lidar(msg); });

        // 切换到 gamepad 控制模式
        auto pub = create_publisher<cyberdog_msg::msg::YamlParam>("yaml_parameter", 10);
        auto param = cyberdog_msg::msg::YamlParam();
        param.name = "use_rc";
        param.kind = 2;
        param.s64_value = 0;
        param.is_user = 0;
        rclcpp::sleep_for(std::chrono::seconds(1));
        pub->publish(param);

        // 自动起立：recovery站起来 → 等稳定 → locomotion切行走模式
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        motion_.recovery();
        rclcpp::sleep_for(std::chrono::seconds(2));
        motion_.locomotion();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        motion_.set_pitch(0.4f);  // 低头15度看地面黄线

        // 初始化各赛段
        stages_[0] = std::make_unique<Stage1>(motion_, sensor_);
        stages_[1] = std::make_unique<Stage2>(motion_, sensor_);
        stages_[2] = std::make_unique<Stage3>(motion_, sensor_);
        stages_[3] = std::make_unique<Stage4>(motion_, sensor_);
        stages_[4] = std::make_unique<Stage5>(motion_, sensor_);
        stages_[5] = std::make_unique<Stage6>(motion_, sensor_);

        stages_[cur_stage_]->init();

        // LCM 订阅 simulator_state，获取里程计位置
        if (lcm_sub_.good()) {
            lcm_sub_.subscribe("simulator_state", &RaceController::on_sim_state, this);
            lcm_running_ = true;
            lcm_thread_ = std::thread([this]() {
                while (lcm_running_) {
                    lcm_sub_.handleTimeout(100);
                }
            });
        } else {
            RCLCPP_WARN(get_logger(), "LCM init failed, odom will be unavailable");
        }

        // 主控制循环 100Hz
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { control_loop(); });

        RCLCPP_INFO(get_logger(), "Race controller started, stage %d", cur_stage_ + 1);
    }

    // 供外部信号处理访问
    MotionCtrl& get_motion() { return motion_; }

    ~RaceController() {
        lcm_running_ = false;
        if (lcm_thread_.joinable()) lcm_thread_.join();
    }

private:
    void on_rgb(sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_img = cv_bridge::toCvShare(msg, "bgr8");
        std::lock_guard<std::mutex> lock(vision_mutex_);

        auto lane = lane_detector_.detect(cv_img->image);
        auto ball = ball_detector_.detect(cv_img->image, BallColor::ORANGE);
        sensor_.lane_offset    = lane.offset;
        sensor_.lane_yaw       = lane.yaw;
        sensor_.lane_curvature = lane.curvature;
        sensor_.lane_valid     = lane.valid;
        sensor_.ball_found  = ball.found;
        sensor_.ball_x      = ball.cx;
        sensor_.ball_dist   = ball.radius;

#ifdef DEBUG_VISION
        // 视觉可视化：黄线边界 + 中心线 + 球检测
        cv::Mat frame = cv_img->image.clone();
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(20, 100, 150), cv::Scalar(35, 255, 255), mask);
        cv::Mat overlay = frame.clone();
        overlay.setTo(cv::Scalar(0, 255, 0), mask);
        cv::addWeighted(frame, 0.7, overlay, 0.3, 0, frame);

        for (auto& p : lane_detector_.last_left_)
            cv::circle(frame, p, 4, {255, 0, 0}, -1);
        for (auto& p : lane_detector_.last_right_)
            cv::circle(frame, p, 4, {0, 0, 255}, -1);

        // 中心线（黄色点）
        auto& lpts = lane_detector_.last_left_;
        auto& rpts = lane_detector_.last_right_;
        for (size_t li = 0, ri = 0; li < lpts.size() && ri < rpts.size(); ) {
            if (lpts[li].y == rpts[ri].y) {
                cv::circle(frame, {(lpts[li].x + rpts[ri].x) / 2, lpts[li].y}, 3, {0, 255, 255}, -1);
                li++; ri++;
            } else if (lpts[li].y > rpts[ri].y) { li++; } else { ri++; }
        }

        // 单边丢失时：画估算的虚拟对侧边界（品红色虚线）
        if (lane.valid && lane.lane_width > 0) {
            if (lpts.empty() && !rpts.empty()) {
                // 只有右边界，估算左边界
                for (auto& p : rpts) {
                    int est_x = static_cast<int>(p.x - lane.lane_width);
                    if (est_x >= 0)
                        cv::circle(frame, {est_x, p.y}, 3, {255, 0, 255}, -1);
                }
            } else if (rpts.empty() && !lpts.empty()) {
                // 只有左边界，估算右边界
                for (auto& p : lpts) {
                    int est_x = static_cast<int>(p.x + lane.lane_width);
                    if (est_x < frame.cols)
                        cv::circle(frame, {est_x, p.y}, 3, {255, 0, 255}, -1);
                }
            }
        }

        // 球检测框
        if (ball.found) {
            int bx = static_cast<int>((ball.cx + 1.0f) / 2.0f * frame.cols);
            int by = static_cast<int>((ball.cy + 1.0f) / 2.0f * frame.rows);
            cv::circle(frame, {bx, by}, static_cast<int>(ball.radius), {0, 165, 255}, 2);
        }

        // 图像中心线 + 文字
        cv::line(frame, {frame.cols/2, 0}, {frame.cols/2, frame.rows}, {255, 255, 255}, 1);
        cv::putText(frame,
            cv::format("S%d | offset=%.2f curv=%.1f w=%.0fpx ball=%d dist=%.2f",
                cur_stage_+1, lane.offset, lane.curvature, lane.lane_width, ball.found, ball.radius),
            {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 255}, 2);
        cv::putText(frame,
            cv::format("odom x=%.3f y=%.3f yaw=%.3f",
                sensor_.odom_x, sensor_.odom_y, sensor_.yaw),
            {10, 58}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 255}, 2);
        cv::imshow("RaceDebug", frame);
        cv::waitKey(1);
#endif
    }

    void on_sim_state(const lcm::ReceiveBuffer*, const std::string&,
                      const simulator_lcmt* msg) {
        sensor_.odom_x = static_cast<float>(msg->p[0]);
        sensor_.odom_y = static_cast<float>(msg->p[1]);
#ifdef DEBUG_SENSOR
        RCLCPP_DEBUG(get_logger(), "[Odom] x=%.3f y=%.3f",
                     sensor_.odom_x, sensor_.odom_y);
#endif
    }

    void on_imu(sensor_msgs::msg::Imu::SharedPtr msg) {
        // 四元数转欧拉角
        auto& q = msg->orientation;
        sensor_.yaw   = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                   1.0 - 2.0*(q.y*q.y + q.z*q.z));
        sensor_.pitch = std::asin(2.0*(q.w*q.y - q.z*q.x));
        sensor_.roll  = std::atan2(2.0*(q.w*q.x + q.y*q.z),
                                   1.0 - 2.0*(q.x*q.x + q.y*q.y));

#ifdef DEBUG_SENSOR
        RCLCPP_DEBUG(get_logger(), "[IMU] yaw=%.3f pitch=%.3f roll=%.3f",
                     sensor_.yaw, sensor_.pitch, sensor_.roll);
#endif
    }

    void on_lidar(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg->ranges.empty()) return;
        int n = msg->ranges.size();
        float front_min = 10.0f;
        for (int i = n/3; i < 2*n/3; i++) {
            if (msg->ranges[i] < front_min) front_min = msg->ranges[i];
        }
        sensor_.lidar_front = front_min;

#ifdef DEBUG_SENSOR
        RCLCPP_DEBUG(get_logger(), "[Lidar] front=%.2f", sensor_.lidar_front);
#endif
    }

    void control_loop() {
        if (cur_stage_ >= 6) return;

        stages_[cur_stage_]->run();

        if (stages_[cur_stage_]->is_done()) {
#ifdef DEBUG_STAGE
            RCLCPP_INFO(get_logger(), "[Stage] %d done", cur_stage_ + 1);
#endif
            // 单赛段模式：完成后停止，不切换
            if (single_stage_mode_) {
                motion_.stop();
                RCLCPP_WARN(get_logger(), "[DEBUG] Stage %d done, stopped (single stage mode)", cur_stage_ + 1);
                timer_->cancel();
                return;
            }

#ifdef DEBUG_END_STAGE
            if (cur_stage_ + 1 >= DEBUG_END_STAGE) {
                motion_.stop();
                RCLCPP_WARN(get_logger(), "[DEBUG] Stage %d done, stopped (end stage mode)", cur_stage_ + 1);
                timer_->cancel();
                return;
            }
#endif

            cur_stage_++;
            if (cur_stage_ < 6) {
#ifdef DEBUG_STAGE
                RCLCPP_INFO(get_logger(), "[Stage] switching to stage %d", cur_stage_ + 1);
#endif
                stages_[cur_stage_]->init();
            } else {
                RCLCPP_INFO(get_logger(), "All stages complete!");
            }
        }
    }

    MotionCtrl  motion_;
    SensorData  sensor_;
    std::mutex  vision_mutex_;

    lcm::LCM    lcm_sub_;
    std::thread lcm_thread_;
    std::atomic<bool> lcm_running_{false};

    LaneDetector lane_detector_;
    BallDetector ball_detector_;

    int  cur_stage_{0};
    bool single_stage_mode_{false};
    std::unique_ptr<StageBase> stages_[6];

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr     sub_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RaceController>();
    g_motion = &node->get_motion();
    std::signal(SIGINT,  on_shutdown);
    std::signal(SIGTERM, on_shutdown);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
