#pragma once
#include <rclcpp/rclcpp.hpp>
#include "cyberdog_race/stages/stage_base.hpp"
#include "cyberdog_race/vision/virtual/stage4_detector.hpp"

// 第四赛段：深隧寻珍
// 路径点导航 → 蹲下识别足球 → 蛇形路径 → 识别蓝球 → 蹲下穿越 → 退出
class Stage4 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override;
    float get_desired_height() const override;
    float get_desired_step_height() const override;

    Stage4Result vision_result;
    bool crouch_active{false};
    static constexpr float CROUCH_HEIGHT = 0.15f;  // 蹲下目标高度15cm

private:
    bool done_{false};
    int  wp_idx_{0};
    int  crouch_frames_{0};
    int  detect_wait_{0};
    float crouch_start_x_{0.f};
    float crouch_start_y_{0.f};
    rclcpp::Time detect_start_time_;

    enum class State {
        MOVE_TO_WP,       // 走路径点
        TURN_TO_Y,        // WP0到达后转向y正方向
        CROUCH_PASS,      // 蹲下等到位
        DETECT_FOOTBALL,  // 识别足球
        ADVANCE,          // 识别到足球后前进1m
        RETREAT,          // 后退到y=9.7
        RECOVER_HEIGHT,   // 起立
        EXIT_TURN,        // 转向y负方向
        EXIT_MOVE,        // 走到y=9.0
        EXIT_TURN2,       // 转向x负方向
        EXIT_MOVE2,       // 走到x=1.75
        EXIT_TURN3,       // 转向-x偏+y 30度
        EXIT_MOVE3,       // 前进0.3m
        EXIT_TURN4,       // 转向y正方向
        EXIT_MOVE4,       // 走到(0.9,9.75)
        DETECT_BLUE,      // 识别蓝球
        ADVANCE_BLUE,     // 识别到蓝球后前进1m
        EXIT_TURN5,       // 转向y负方向
        EXIT_MOVE5,       // 走到(0.9,9.15)
        EXIT_TURN6,       // 转向-x偏-y 15度
        EXIT_MOVE6,       // 走到(0,9.0)
        EXIT_TURN7,       // 转向y正方向，蹲下
        CROUCH_PASS2,     // 第二次蹲下
        ADVANCE2,         // 蹲着前进1m
        RECOVER_HEIGHT2,  // 起立
        ADVANCE3,         // 前进0.5m
        FINAL_TURN,       // 转向y负方向
        FINAL_MOVE,       // 前进0.5m
        CROUCH_PASS3,     // 第三次蹲下
        ADVANCE4,         // 蹲着前进1m
        RECOVER_HEIGHT3,  // 起立
        FINAL_MOVE2,      // 走到(0.5,7.35)
        FINAL_TURN2,      // 转向x正方向
        FINAL_MOVE3,      // 走到(2.9,7.35)
        FINAL_TURN3,      // 转向y正方向
        DONE_WAIT
    } state_{State::MOVE_TO_WP};

    struct WP { float x, y; };
    static constexpr int NUM_WP = 2;
    static constexpr WP WAYPOINTS[NUM_WP] = {
        {2.25f, 7.35f},
        {2.15f, 9.75f},
    };
    static constexpr float WP_THRESH = 0.3f;
    static constexpr float SPEED     = 0.82f;
    static constexpr float KP_IMU    = 0.8f;
};
