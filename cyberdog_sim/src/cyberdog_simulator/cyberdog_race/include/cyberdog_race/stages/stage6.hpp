#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第六赛段：撷金建功
// 起立 → 转向x负 → 走到(2.35,13.0) → 转向y正 → 走到(2.35,14.75) → 转向x负
// → 走到(0.35,15.0) → 转向纯y负 → 走到(0.15,13.0)
// → 慢速转向x正 → 走到(2.35,13.0) → 趴下
class Stage6 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    bool is_done() override;
    bool needs_rc_mode() const { return rc_mode_needed_; }

private:
    bool done_{false};
    bool rc_mode_needed_{false};
    int  turn_frames_{0};

    static constexpr float SPEED          = 0.7f;
    static constexpr float KP_IMU         = 2.0f;

    // 路径点位
    static constexpr float WP1_X = 2.35f;   // 第一个路点 X
    static constexpr float WP1_Y = 13.0f;   // 第一个路点 Y
    static constexpr float WP2_X = 2.35f;   // 第二个路点 X
    static constexpr float WP2_Y = 14.75f;  // 第二个路点 Y
    static constexpr float WP3_X = 0.35f;   // 第三个路点 X
    static constexpr float WP3_Y = 15.0f;   // 第三个路点 Y
    static constexpr float WP4_X = 0.25f;   // 第四个路点 X
    static constexpr float WP4_Y = 13.15f;   // 第四个路点 Y
    static constexpr float WP5_X = 2.35f;   // 第五个路点 X
    static constexpr float WP5_Y = 13.0f;   // 第五个路点 Y

    enum class State {
        STAND,              // 起立
        TURN_X_NEG_INIT,    // 原地转向x负方向
        GO_TO_WP1,          // 走到(2.35,13.0)
        TURN_Y_POS,         // 原地转向y正方向
        GO_TO_WP2,          // y正向走到(2.35,14.75)
        TURN_X_NEG,         // 原地转向x负方向
        GO_TO_WP3,          // x负向走到(0.35,15.0)
        TURN_Y_NEG,         // 原地转向西偏南10°
        GO_TO_WP4,          // 走到(0.15,13.0)
        TURN_X_POS_SLOW,    // 慢速转向x正方向
        GO_TO_WP5,          // 走到(2.35,13.0)
        FINAL_LIE_DOWN      // 最后趴下
    } state_{State::STAND};
};
