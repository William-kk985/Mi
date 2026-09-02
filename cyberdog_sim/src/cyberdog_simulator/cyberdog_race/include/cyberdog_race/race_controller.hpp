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
#include "cyberdog_race/utils/sensor_data.hpp"
#include "cyberdog_race/stages/stage_base.hpp"
#include "cyberdog_race/stages/virtual/stage1.hpp"
#include "cyberdog_race/stages/virtual/stage2.hpp"
#include "cyberdog_race/stages/virtual/stage3.hpp"
#include "cyberdog_race/stages/virtual/stage4.hpp"
#include "cyberdog_race/stages/virtual/stage5.hpp"
#include "cyberdog_race/stages/virtual/stage6.hpp"
#ifdef REAL_DOG
#include "cyberdog_race/stages/real/stage1_real.hpp"  // 真机第1赛段 石径探路
#include "cyberdog_race/stages/real/stage2_real.hpp"  // 真机第2赛段
#include "cyberdog_race/stages/real/stage3_real.hpp"  // 真机第3赛段 破限低头前进
#include "cyberdog_race/stages/real/stage4_real.hpp"  // 真机第4赛段 伙伴逻辑
#include "cyberdog_race/stages/real/stage5_real.hpp"  // 真机第5赛段 伙伴逻辑
#include "cyberdog_race/stages/real/stage6_real.hpp"  // 真机第6赛段 伙伴逻辑
#include "stage4_real_test.hpp"  // ★ Stage4 test 路线实现，无条件编入 + launch 参数选择
#include "stage4_real_test2.hpp"  // ★ Stage4 test2 路线实现，无条件编入 + launch 参数选择
#include "stage2_real_test.hpp"  // 测试版 Stage2：旧侧移扫球逻辑
#endif
#ifdef USE_TEST_REAL_STAGE3
#include "stage3_real_test.hpp"  // 测试版 Stage3：伙伴算法实验
#endif
#endif
#include "behavior_test.hpp"
#ifdef USE_TEST_LANE_V2
#include "lane_detector_v2.hpp"   // 伙伴连通域寻线实验版
#else
#include "cyberdog_race/vision/virtual/lane_detector.hpp"
#endif
#include "cyberdog_race/vision/virtual/ball_detector.hpp"
#include "cyberdog_race/vision/vision_result.hpp"   // YoloResult 公共结果结构
#ifdef REAL_DOG
#include "cyberdog_race/vision/real/yolo_detector.hpp"   // 真机多目标检测（socket YOLO + CV）
#include "cyberdog_race/vision/real/lidar_locator.hpp"   // 雷达定位（样子）
#endif
#include "cyberdog_race/utils/web_streamer.hpp"
#include "cyberdog_race/utils/llm_helper.hpp"

class RaceController : public rclcpp::Node {
public:
    RaceController();
    ~RaceController();
    MotionCtrl& get_motion() { return motion_; }
    void on_external_imu(const lcm::ReceiveBuffer*, const std::string&);  // 原始包解码 quat，public 供静态回调桥调用

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
    void on_odom_out(nav_msgs::msg::Odometry::SharedPtr msg);  // RT 板腿里程计航向
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
#ifdef REAL_DOG
    YoloDetector yolo_detector_;     // 真机多目标检测（2026-09-02: 替代仿真 Stage4Detector）
    LidarLocator lidar_locator_;     // 雷达定位（样子）
#endif

    unsigned rgb_recv_cnt_{0};   // on_rgb 累计收帧数（订阅匹配自检用）

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
    rclcpp::TimerBase::SharedPtr diag_timer_;   // RGB 自检 timer（必须保存否则被析构）
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_out_;  // RT 板腿里程计
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
