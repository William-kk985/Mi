#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第五赛段：过桥 → 走完全程
class Stage5 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override;
    bool needs_rc_mode() const override { return rc_mode_needed_; }
    float get_desired_height() const override;
    float get_desired_roll() const override;
    float get_desired_step_height() const override;

    // main.cpp 兼容字段（保持默认值即可）
    bool  crouch_active{false};
    bool  roll_active{false};
    float target_roll{0.0f};
    static constexpr float CROUCH_HEIGHT = 0.22f;

private:
    bool done_{false};
    bool rc_mode_needed_{false};
    int  turn_frames_{0};
    int  jump_frames_{0};

    enum class State {
        MOVE_NORTH_1,
        TURN_LEFT_1,
        MOVE_WEST,
        TURN_RIGHT_1,
        MOVE_NORTH_2,
        TURN_RIGHT_2,
        MOVE_EAST,
        TURN_RIGHT_3,
        MOVE_SOUTH,
        JUMP_PREPARE,
        JUMP,
        JUMP_EXECUTE
    } state_{State::MOVE_NORTH_1};
};
