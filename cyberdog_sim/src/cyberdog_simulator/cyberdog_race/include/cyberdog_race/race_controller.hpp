#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cyberdog_msg/msg/yaml_param.hpp>
#include <cv_bridge/cv_bridge.h>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <cmath>

#include "simulator_lcmt.hpp"
#include "cyberdog_race/debug_config.hpp"
#include "cyberdog_race/motion_ctrl.hpp"
#include "cyberdog_race/sensor_data.hpp"
#include "cyberdog_race/stages/stage_base.hpp"
#include "cyberdog_race/stages/virtual/stage1.hpp"
#include "cyberdog_race/stages/virtual/stage2.hpp"
#include "cyberdog_race/stages/virtual/stage3.hpp"
#include "cyberdog_race/stages/virtual/stage4.hpp"
#include "cyberdog_race/stages/virtual/stage5.hpp"
#include "cyberdog_race/stages/virtual/stage6.hpp"
#include "cyberdog_race/behavior_test.hpp"
#include "cyberdog_race/vision/virtual/lane_detector.hpp"
#include "cyberdog_race/vision/virtual/ball_detector.hpp"
#include "cyberdog_race/vision/virtual/stage4_detector.hpp"

class RaceController : public rclcpp::Node {
public:
    RaceController();
    ~RaceController();
    MotionCtrl& get_motion() { return motion_; }

private:
    void on_rgb(sensor_msgs::msg::Image::SharedPtr msg);
    void on_imu(sensor_msgs::msg::Imu::SharedPtr msg);
    void on_lidar(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void on_sim_state(const lcm::ReceiveBuffer*, const std::string&,
                      const simulator_lcmt* msg);
    void control_loop();
    void apply_stage_params();

    SensorData read_sensor_snapshot();

    MotionCtrl  motion_;
    SensorData  sensor_;
    std::mutex  sensor_mutex_;

    lcm::LCM    lcm_sub_;
    std::thread lcm_thread_;
    std::atomic<bool> lcm_running_{false};

    LaneDetector lane_detector_;
    BallDetector ball_detector_;
    Stage4Detector stage4_detector_;

    int  cur_stage_{0};
    bool single_stage_mode_{false};
    // 参数变化检测（避免每帧重复发送）
    bool last_rc_mode_{false};
    float last_sent_height_{0.0f};
    float last_sent_step_h_{0.0f};
    int extra_params_stage_{-1};  // 已发 extra_params 的赛段索引
    std::unique_ptr<StageBase> stages_[6];

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr     sub_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;
    rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr   yaml_pub_;
};
