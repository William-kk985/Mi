#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cyberdog_msg/msg/yaml_param.hpp>
#include <cv_bridge/cv_bridge.h>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <cmath>
#include <unistd.h>

#include "simulator_lcmt.hpp"
#include "cyberdog_race/debug_config.hpp"
#include "cyberdog_race/motion_ctrl.hpp"
#include "cyberdog_race/vision/lane_detector.hpp"
#include "cyberdog_race/vision/ball_detector.hpp"
#include "cyberdog_race/vision/stage4_detector.hpp"
#include "cyberdog_race/stages/stage_base.hpp"
#include "cyberdog_race/stages/stage1.hpp"
#include "cyberdog_race/stages/stage2.hpp"
#include "cyberdog_race/stages/stage3.hpp"
#include "cyberdog_race/stages/stage4.hpp"
#include "cyberdog_race/stages/stage5.hpp"
#include "cyberdog_race/stages/stage6.hpp"

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
        yaml_pub_ = create_publisher<cyberdog_msg::msg::YamlParam>("yaml_parameter", 10);
        auto param = cyberdog_msg::msg::YamlParam();
        param.name = "use_rc";
        param.kind = 2;
        param.s64_value = 0;
        param.is_user = 0;
        rclcpp::sleep_for(std::chrono::seconds(1));
        yaml_pub_->publish(param);

        // 自动起立：recovery站起来 → 等稳定 → locomotion切行走模式
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        motion_.recovery();
        rclcpp::sleep_for(std::chrono::seconds(2));
        motion_.locomotion();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        motion_.set_pitch(-0.26f);  // 低头15度看地面黄线

        // 初始化各赛段
        stages_[0] = std::make_unique<Stage1>(motion_, sensor_);
        stages_[1] = std::make_unique<Stage2>(motion_, sensor_);
        stages_[2] = std::make_unique<Stage3>(motion_, sensor_);
        stages_[3] = std::make_unique<Stage4>(motion_, sensor_);
        stages_[4] = std::make_unique<Stage5>(motion_, sensor_);
        stages_[5] = std::make_unique<Stage6>(motion_, sensor_);

        // 根据起始赛段设置视觉模式
        if (cur_stage_ == 2) {
            lane_detector_.set_mode(LaneMode::RELAXED);
        }
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

    ~RaceController() {
        lcm_running_ = false;
        if (lcm_thread_.joinable()) lcm_thread_.join();
    }

    MotionCtrl& get_motion() { return motion_; }

private:
    void on_rgb(sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_img = cv_bridge::toCvShare(msg, "bgr8");
        std::lock_guard<std::mutex> lock(vision_mutex_);

        auto lane = lane_detector_.detect(cv_img->image);
        auto ball = ball_detector_.detect(cv_img->image, BallColor::ORANGE);
        auto blue = ball_detector_.detect(cv_img->image, BallColor::BLUE);
        sensor_.lane_offset    = lane.offset;
        sensor_.lane_curvature = lane.curvature;
        sensor_.lane_valid     = lane.valid;
        sensor_.lane_both_sides = lane.both_sides;
        sensor_.ball_found  = ball.found;
        sensor_.ball_x      = ball.cx;
        sensor_.ball_dist   = ball.dist_m;
        sensor_.blue_ball_found = blue.found;
        sensor_.blue_ball_x     = blue.cx;
        sensor_.blue_ball_dist  = blue.dist_m;

        // stage4 视觉检测
        if (cur_stage_ == 3) {
            auto s4 = static_cast<Stage4*>(stages_[3].get());
            s4->vision_result = stage4_detector_.detect(cv_img->image);
        }

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

        // 球检测框 + 距离标注
        if (ball.found) {
            int bx = static_cast<int>((ball.cx + 1.0f) / 2.0f * frame.cols);
            int by = static_cast<int>((ball.cy + 1.0f) / 2.0f * frame.rows);
            cv::circle(frame, {bx, by}, static_cast<int>(ball.radius), {0, 165, 255}, 2);
            // 在球旁边显示距离
            cv::putText(frame,
                cv::format("r=%.0fpx d=%.2fm", ball.radius, ball.dist_m),
                {bx + 5, by - 5}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 165, 255}, 1);
        }

        // 图像中心线 + 文字
        cv::line(frame, {frame.cols/2, 0}, {frame.cols/2, frame.rows}, {255, 255, 255}, 1);
        cv::putText(frame,
            cv::format("S%d | off=%.2f curv=%.1f ball=%d r=%.0fpx dist=%.2fm",
                cur_stage_+1, lane.offset, lane.curvature, ball.found, ball.radius, ball.dist_m),
            {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 255}, 2);
        cv::putText(frame,
            cv::format("odom x=%.3f y=%.3f yaw=%.3f",
                sensor_.odom_x, sensor_.odom_y, sensor_.yaw),
            {10, 58}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 255}, 2);

        // stage4 视觉检测结果可视化
        if (cur_stage_ == 3) {
            auto s4 = static_cast<Stage4*>(stages_[3].get());
            auto& r = s4->vision_result;

            // 足球（白球）：优先显示，画圆圈+距离，跟橙色球一样
            if (r.football_found) {
                int fx = static_cast<int>((r.football_cx + 1.0f) / 2.0f * frame.cols);
                int fy = frame.rows / 2;
                cv::circle(frame, {fx, fy}, 20, {255, 255, 255}, 2);
                cv::line(frame, {fx, 0}, {fx, frame.rows}, {255, 255, 255}, 1);
                cv::putText(frame,
                    cv::format("football d=%.2fm", r.football_dist),
                    {fx + 5, fy - 25}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 2);
            }

            // 其他目标：左侧状态列表
            int line_y = 86;
            auto draw_status = [&](bool found, float dist, const std::string& label, cv::Scalar color) {
                cv::putText(frame,
                    cv::format("%s:%s d=%.2f", label.c_str(), found ? "Y" : "N", dist),
                    {10, line_y}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    found ? color : cv::Scalar(100, 100, 100), 1);
                line_y += 22;
            };
            draw_status(r.football_found, r.football_dist, "football", {255, 255, 255});
            draw_status(r.limbar_found,   r.limbar_dist,   "limbar",   {180, 180, 180});
            draw_status(r.coke_found,     r.coke_dist,     "coke",     {80,  80,  80 });
            draw_status(r.obstacle_found, r.obstacle_dist, "obstacle", {255, 128, 0  });
        }
        cv::imshow("RaceDebug", frame);
        cv::waitKey(1);
#endif
    }

    void on_sim_state(const lcm::ReceiveBuffer*, const std::string&,
                      const simulator_lcmt* msg) {
        sensor_.odom_x = static_cast<float>(msg->p[0]);
        sensor_.odom_y = static_cast<float>(msg->p[1]);
        sensor_.body_height = static_cast<float>(msg->p[2]);
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

        // stage4 蹲下/恢复身高（目标高度变化后持续重复发送，确保控制器收到）
        if (cur_stage_ == 3 && yaml_pub_) {
            auto s4 = static_cast<Stage4*>(stages_[3].get());
            float target_h      = s4->crouch_active ? Stage4::CROUCH_HEIGHT : 0.25f;
            float target_step_h = s4->crouch_active ? 0.03f : 0.20f;  // 蹲下时步高降到3cm
            static float last_height = -1.f;
            static int send_counter = 0;
            if (std::abs(target_h - last_height) > 0.01f) {
                last_height = target_h;
                send_counter = 0;
                fprintf(stderr, "\033[1;33m[Main] 切换身体高度: %.2f 步高: %.2f\033[0m\n", target_h, target_step_h);
            }
            // 高度切换后持续发送30帧，保证控制器收到
            if (send_counter < 30) {
                send_counter++;
                // 发送身体高度
                cyberdog_msg::msg::YamlParam p;
                p.name = "des_roll_pitch_height";
                p.kind = 3;
                p.is_user = 1;
                p.vecxd_value[0] = 0.0;
                p.vecxd_value[1] = 0.0;
                p.vecxd_value[2] = target_h;
                yaml_pub_->publish(p);
                // 发送步高上限
                cyberdog_msg::msg::YamlParam p2;
                p2.name = "step_height_max";
                p2.kind = 1;  // kDOUBLE
                p2.is_user = 1;
                p2.double_value = target_step_h;  // kind=DOUBLE 必须用 double_value
                yaml_pub_->publish(p2);
            }
        }

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
                // 根据赛段切换视觉检测模式
                if (cur_stage_ == 2) {  // stage3（0-indexed=2）用宽松模式
                    lane_detector_.set_mode(LaneMode::RELAXED);
                } else {
                    lane_detector_.set_mode(LaneMode::STRICT);
                }
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
    Stage4Detector stage4_detector_;

    int  cur_stage_{0};
    bool single_stage_mode_{false};
    std::unique_ptr<StageBase> stages_[6];

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr     sub_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;
    rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr   yaml_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RaceController>();

    rclcpp::spin(node);

    // Ctrl+C后发停止指令再坐下
    auto& motion = node->get_motion();
    for (int i = 0; i < 20; i++) {
        motion.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stand();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    motion.lie_down();

    rclcpp::shutdown();
    return 0;
}
