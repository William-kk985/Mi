#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第六赛段真机版：按起点朝向执行直行、转向和左后斜行，最后趴下。
class Stage6Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override;

    float get_desired_height() const override { return kBodyHeight; }
    float get_desired_step_height() const override { return kStepHeight; }

private:
    enum class State {
        MOVE_FORWARD_1,
        TURN_RIGHT_48,
        MOVE_FORWARD_2,
        TURN_LEFT_48,
        MOVE_FORWARD_3,
        MOVE_LEFT_BACK_41,
        MOVE_BACKWARD,
        LIE_DOWN,
        DONE
    };

    void begin_walk(State state, float local_x, float local_y,
                    float distance, float target_yaw);
    bool update_walk();
    void begin_turn(State state, float target_yaw);
    bool update_turn();

    State state_{State::MOVE_FORWARD_1};
    bool done_{false};
    bool pose_initialized_{false};
    bool lie_sent_{false};
    int state_frames_{0};
    int stable_frames_{0};

    // 起点地图朝向；local x=前方，local y=左侧（对应 vel_des.y > 0）。
    float start_yaw_{0.0f};
    float target_yaw_{0.0f};
    float segment_x_{0.0f};
    float segment_y_{0.0f};
    float segment_dir_x_{0.0f};
    float segment_dir_y_{0.0f};
    float segment_distance_{0.0f};

    static constexpr float kBodyHeight = 0.25f;
    static constexpr float kStepHeight = 0.10f;
    static constexpr float kSpeed = 0.15f;
    static constexpr float kYawKp = 0.8f;
    static constexpr float kYawMax = 0.25f;
    static constexpr float kTurnYawMax = 0.38f;
    static constexpr float kYawTolerance = 0.045f;
    static constexpr float kDistanceTolerance = 0.035f;
    static constexpr int kTurnStableFrames = 15;
    static constexpr int kWalkTimeoutFrames = 5000;
    static constexpr int kTurnTimeoutFrames = 1200;
};
