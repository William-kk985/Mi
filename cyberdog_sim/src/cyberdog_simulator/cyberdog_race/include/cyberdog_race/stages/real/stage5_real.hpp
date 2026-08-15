#pragma once

#include "cyberdog_race/stages/stage_base.hpp"

// 第五赛段：连续通过五段独木桥并跳入桥围成的内部区域。
class Stage5Real : public StageBase {
public:
    using StageBase::StageBase;

    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override;

    float get_desired_height() const override;
    float get_desired_roll() const override;
    float get_desired_step_height() const override;

private:
    enum class State {
        WALK_SECTION_1,
        TURN_LEFT,
        WALK_SECTION_2,
        TURN_RIGHT_1,
        WALK_SECTION_3,
        TURN_RIGHT_2,
        WALK_SECTION_4,
        TURN_RIGHT_3,
        WALK_SECTION_5,
        TURN_TO_DROP,
        JUMP_PREPARE,
        JUMP,
        JUMP_SETTLE,
        DONE
    };

    void begin_walk(State next_state, float distance, float yaw_offset, bool tilted);
    bool update_walk();
    void begin_turn(State next_state, float yaw_offset);
    bool update_turn();
    float projected_distance() const;

    State state_{State::WALK_SECTION_1};
    State next_state_{State::WALK_SECTION_1};

    float start_yaw_{0.0f};
    float target_yaw_{0.0f};
    float target_distance_{0.0f};
    float segment_start_x_{0.0f};
    float segment_start_y_{0.0f};
    float desired_roll_{0.0f};

    int stable_frames_{0};
    int state_frames_{0};
    bool done_{false};
};

