#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第一赛段：石径探路
// 黄线巡线 + 弯道检测 + 石板路步态
class Stage1 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    bool is_done() override;

private:
    bool  done_{false};
    float yaw_start_{0.0f};
    float prev_offset_{0.0f};
    bool  in_turn_{false};
    bool  at_junction_{false};
    int   run_frames_{0};
    int   lane_lost_frames_{0};

    // 卡住检测
    float last_odom_x_{0.0f};
    float last_odom_y_{0.0f};
    int   stuck_frames_{0};
    static constexpr int   STUCK_THRESH   = 30;   // 连续30帧（0.3s）没动视为卡住
    static constexpr float STUCK_DIST     = 0.01f; // 移动距离小于1cm视为没动
    static constexpr int   ESCAPE_FRAMES  = 40;   // 脱困持续帧数
    int   escape_frames_{0};
};
