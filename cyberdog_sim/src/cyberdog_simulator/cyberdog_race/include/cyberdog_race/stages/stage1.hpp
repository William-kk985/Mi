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
    int   escape_frames_{0};

    // 控制参数
    static constexpr float KP_YAW        = 0.6f;
    static constexpr float KD_YAW        = 0.08f;
    static constexpr float BASE_SPEED    = 0.45f;
    static constexpr float TURN_SPEED    = 0.2f;
    static constexpr float CURVE_THRESH  = 60.0f;

    // 路口检测
    static constexpr float JUNCTION_X_MIN = 2.95f;
    static constexpr float JUNCTION_X_MAX = 3.2f;

    // 卡住检测参数
    static constexpr int   STUCK_THRESH  = 30;
    static constexpr float STUCK_DIST    = 0.01f;
    static constexpr int   ESCAPE_FRAMES = 40;
};
