#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>          // 超声 ultrasonic_payload
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>  // BMS 电池占位（真机需替换为 protocol::msg::BmsStatus）
// #include "protocol/msg/TouchStatus.hpp"      // TODO: 从真狗 bridges 包获取后启用触摸紧急停止
// #include "protocol/msg/BmsStatus.hpp"        // TODO: 从真狗 bridges 包获取后替换 Float32MultiArray
#include <cyberdog_msg/msg/yaml_param.hpp>
#include <cv_bridge/cv_bridge.h>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <cmath>
#include <deque>

#include "simulator_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "localization_lcmt.hpp"
#include "cyberdog_race/debug_config.hpp"
#ifdef REAL_DOG
#include <protocol/msg/head_tof_payload.hpp>
#include <protocol/msg/rear_tof_payload.hpp>
#endif
#include "cyberdog_race/motion_ctrl.hpp"
#include "cyberdog_race/sensor_data.hpp"
#include "cyberdog_race/stages/stage_base.hpp"
#include "cyberdog_race/stages/virtual/stage1.hpp"
#include "cyberdog_race/stages/virtual/stage2.hpp"
#include "cyberdog_race/stages/virtual/stage3.hpp"
#include "cyberdog_race/stages/virtual/stage4.hpp"
#include "cyberdog_race/stages/virtual/stage5.hpp"
#include "cyberdog_race/stages/virtual/stage6.hpp"
#ifdef REAL_DOG
#include "cyberdog_race/stages/real/stage1_real.hpp"  // 真机第1赛段 石径探路 (2026-08-11)
#endif
#include "behavior_test.hpp"
#include "cyberdog_race/vision/virtual/lane_detector.hpp"
#include "cyberdog_race/vision/virtual/ball_detector.hpp"
#include "cyberdog_race/vision/virtual/stage4_detector.hpp"
#include "cyberdog_race/utils/web_streamer.hpp"
#include "cyberdog_race/llm_helper.hpp"

class RaceController : public rclcpp::Node {
public:
    RaceController();
    ~RaceController();
    MotionCtrl& get_motion() { return motion_; }

private:
    void on_rgb(sensor_msgs::msg::Image::SharedPtr msg);
    void on_imu(sensor_msgs::msg::Imu::SharedPtr msg);
    void on_lidar(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void on_d435_infra1(sensor_msgs::msg::Image::SharedPtr msg);  // D430i 左目红外
    void on_d435_infra2(sensor_msgs::msg::Image::SharedPtr msg);  // D430i 右目红外
    void on_d435_depth(sensor_msgs::msg::Image::SharedPtr msg);   // D430i 深度图（mono16→伪彩色）
#ifdef REAL_DOG
    void on_tof_head(protocol::msg::HeadTofPayload::SharedPtr msg);
    void on_tof_rear(protocol::msg::RearTofPayload::SharedPtr msg);
    void on_ultrasonic(sensor_msgs::msg::Range::SharedPtr msg);
#endif
    // TODO: void on_touch(protocol::msg::TouchStatus::SharedPtr msg); // 需bridges包
    void on_bms(std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void on_sim_state(const lcm::ReceiveBuffer*, const std::string&,
                      const simulator_lcmt* msg);
    void on_global_to_robot(const lcm::ReceiveBuffer*, const std::string&,
                            const localization_lcmt* msg);
    void on_state_estimator(const lcm::ReceiveBuffer*, const std::string&,
                            const state_estimator_lcmt* msg);
    void control_loop();
    void apply_stage_params();

    MotionCtrl  motion_;
    SensorData  sensor_;
    std::mutex  sensor_mutex_;

    lcm::LCM    lcm_sub_;
    std::thread lcm_thread_;
    std::atomic<bool> lcm_running_{false};

    LaneDetector lane_detector_;
    BallDetector ball_detector_;
    Stage4Detector stage4_detector_;

#ifdef ENABLE_WEB_STREAMING
    WebStreamer web_streamer_;
#endif

#if defined(LLM_MODE_PROXY) || defined(LLM_MODE_API)
    LLMHelper llm_;
#endif

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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr     sub_d435_infra1_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr     sub_d435_infra2_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr     sub_d435_depth_;
#ifdef REAL_DOG
    rclcpp::Subscription<protocol::msg::HeadTofPayload>::SharedPtr sub_head_tof_;
    rclcpp::Subscription<protocol::msg::RearTofPayload>::SharedPtr sub_rear_tof_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr      sub_ultrasonic_;
#endif
    // rclcpp::Subscription<protocol::msg::TouchStatus>::SharedPtr sub_touch_; // TODO: 需bridges包
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bms_;
    // rclcpp::Subscription<protocol::msg::HeadTofPayload>::SharedPtr sub_head_tof_;  // TODO: 需bridges包
    // rclcpp::Subscription<protocol::msg::RearTofPayload>::SharedPtr sub_rear_tof_;  // TODO: 需bridges包
    rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr   yaml_pub_;

#ifdef ENABLE_WEB_STREAMING
    std::deque<std::pair<float,float>> odom_history_;  // 轨迹历史 (x,y)
    int track_render_counter_{0};
    int telem_render_counter_{0};
    void render_track_frame();
    void render_telemetry_frame();
    void render_lidar_frame(const std::vector<float>& ranges, float angle_min, float angle_inc, float front_min);
#endif
};
