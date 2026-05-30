#pragma once
#include "cyberdog_race/stages/stage_base.hpp"
#include <cmath>

// 第三赛段：曲道冲锋
// 路径点导航（里程计）+ 视觉微调
class Stage3 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    bool is_done() override;

private:
    bool done_{false};
    int  wp_idx_{0};
    bool turning_to_y_{false};  // 路径点走完后转向y正方向

    struct WP { float x, y; };
    static constexpr int NUM_WP = 5;
    static constexpr WP WAYPOINTS[NUM_WP] = {
        {0.0f,  5.25f},
        {2.4f,  5.7f},
        {2.4f,  6.0f},
        {2.9f,  6.1f},
        {2.9f,  6.7f},
    };
    static constexpr float WP_THRESH   = 0.3f;
    static constexpr float EXIT_X      = 2.9f;
    static constexpr float EXIT_Y      = 7.35f;
    static constexpr float EXIT_THRESH = 0.4f;

    static constexpr float KP_IMU = 1.20f;   // 路径点方向增益 //0.8 ,1.0，1.10
    static constexpr float KP_VIS = 0.22f;  // 视觉微调权重 //0.15 ,0.18 ， 0.20
    static constexpr float SPEED  = 0.50f; //0.2 ,0.30， 0.35
};
