#include "cyberdog_race/stages/real/stage5_real.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

constexpr float kPi = static_cast<float>(M_PI);

// 赛道图尺寸。转弯时保留少量余量，避免四足踩到独木桥外沿。
// 图纸主路径前四段为 400、400、300、400 cm。转弯前预留约 15 cm，
// 避免四足踩到桥外沿；第五段 150 cm，末端预留 50 cm 用于转向和跳下。
constexpr float kSection1Distance = 3.85f;
constexpr float kSection2Distance = 3.85f;
constexpr float kSection3Distance = 2.85f;
constexpr float kSection4Distance = 3.85f;
constexpr float kSection5Distance = 1.00f;

constexpr float kFlatSpeed = 0.16f;
constexpr float kSlopeSpeed = 0.12f;
constexpr float kSlopeLateralSpeed = 0.02f;  // 后四段左高右低桥：向左微调
constexpr float kStepHeight = 0.10f;
constexpr float kBodyHeight = 0.25f;

// 第二至第五段左高右低。图纸约为50厘米横向跨度、10厘米高差，
// atan(0.10 / 0.50)约等于0.20弧度。
constexpr float kSlopeRoll = 0.20f;

constexpr float kYawTolerance = 0.045f;
constexpr int kTurnStableFrames = 15;
constexpr int kPrepareFrames = 80;
constexpr int kJumpSettleFrames = 180;
constexpr int kWalkTimeoutFrames = 6000;
constexpr int kTurnTimeoutFrames = 1200;

float clamp(float value, float low, float high) {
    return std::max(low, std::min(high, value));
}

}  // 匿名命名空间

void Stage5Real::init() {
    done_ = false;
    state_ = State::WALK_SECTION_1;
    next_state_ = State::WALK_SECTION_1;
    start_yaw_ = sensor_.abs_yaw;
    desired_roll_ = 0.0f;
    stable_frames_ = 0;
    state_frames_ = 0;

    motion_.locomotion();
    motion_.set_body_params_lcm(0.0f, 0.0f, kBodyHeight);
    begin_walk(State::TURN_LEFT, kSection1Distance, 0.0f, false);

    std::fprintf(stderr,
        "[Stage5Real] init odom=(%.3f, %.3f), yaw=%.3f\n",
        sensor_.odom_x, sensor_.odom_y, start_yaw_);
}

void Stage5Real::run() {
    if (done_) return;

    ++state_frames_;

    switch (state_) {
    case State::WALK_SECTION_1:
        if (update_walk()) begin_turn(State::WALK_SECTION_2, kPi / 2.0f);
        break;

    case State::TURN_LEFT:
        if (update_turn()) {
            begin_walk(State::TURN_RIGHT_1, kSection2Distance, kPi / 2.0f, true);
        }
        break;

    case State::WALK_SECTION_2:
        if (update_walk()) begin_turn(State::WALK_SECTION_3, 0.0f);
        break;

    case State::TURN_RIGHT_1:
        if (update_turn()) {
            begin_walk(State::TURN_RIGHT_2, kSection3Distance, 0.0f, true);
        }
        break;

    case State::WALK_SECTION_3:
        if (update_walk()) begin_turn(State::WALK_SECTION_4, -kPi / 2.0f);
        break;

    case State::TURN_RIGHT_2:
        if (update_turn()) {
            begin_walk(State::TURN_RIGHT_3, kSection4Distance, -kPi / 2.0f, true);
        }
        break;

    case State::WALK_SECTION_4:
        if (update_walk()) begin_turn(State::WALK_SECTION_5, -kPi);
        break;

    case State::TURN_RIGHT_3:
        if (update_turn()) {
            begin_walk(State::JUMP_PREPARE, kSection5Distance, -kPi, true);
        }
        break;

    case State::WALK_SECTION_5:
        if (update_walk()) {
            // 最后一段独木桥指向图纸上方。右转90度后再跳，
            // 落入四根独木桥围成的内部区域，而不是越过桥端。
            // 第五段航向为start_yaw-pi，再右转后为start_yaw-3*pi/2，
            // 等价于start_yaw+pi/2。
            begin_turn(State::JUMP_PREPARE, -3.0f * kPi / 2.0f);
            state_ = State::TURN_TO_DROP;
        }
        break;

    case State::TURN_TO_DROP:
        if (update_turn()) {
            state_ = State::JUMP_PREPARE;
            state_frames_ = 0;
            motion_.stop();
            motion_.set_body_params_lcm(kSlopeRoll, 0.0f, kBodyHeight);
        }
        break;

    case State::JUMP_PREPARE:
        motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, kStepHeight);
        if (state_frames_ >= kPrepareFrames) {
            state_ = State::JUMP;
            state_frames_ = 0;
        }
        break;

    case State::JUMP:
        // 前跳60厘米越过剩余50厘米，避免身体撞击独木桥末端。
        motion_.jump_forward(0.60f);
        desired_roll_ = 0.0f;
        motion_.set_body_params_lcm(0.0f, 0.0f, kBodyHeight);
        state_ = State::JUMP_SETTLE;
        state_frames_ = 0;
        break;

    case State::JUMP_SETTLE:
        motion_.stop();
        if (state_frames_ >= kJumpSettleFrames) {
            state_ = State::DONE;
            done_ = true;
            std::fprintf(stderr, "[Stage5Real] completed\n");
        }
        break;

    case State::DONE:
        done_ = true;
        break;
    }
}

bool Stage5Real::is_done() {
    return done_;
}

float Stage5Real::get_desired_height() const {
    return kBodyHeight;
}

float Stage5Real::get_desired_roll() const {
    return desired_roll_;
}

float Stage5Real::get_desired_step_height() const {
    return kStepHeight;
}

void Stage5Real::begin_walk(
    State next_state, float distance, float yaw_offset, bool tilted)
{
    next_state_ = next_state;
    target_distance_ = distance;
    target_yaw_ = norm_yaw(start_yaw_ + yaw_offset);
    segment_start_x_ = sensor_.odom_x;
    segment_start_y_ = sensor_.odom_y;
    desired_roll_ = tilted ? kSlopeRoll : 0.0f;
    stable_frames_ = 0;
    state_frames_ = 0;

    motion_.set_body_params_lcm(desired_roll_, 0.0f, kBodyHeight);

    if (!tilted) state_ = State::WALK_SECTION_1;
    else if (next_state == State::TURN_RIGHT_1) state_ = State::WALK_SECTION_2;
    else if (next_state == State::TURN_RIGHT_2) state_ = State::WALK_SECTION_3;
    else if (next_state == State::TURN_RIGHT_3) state_ = State::WALK_SECTION_4;
    else state_ = State::WALK_SECTION_5;
}

bool Stage5Real::update_walk() {
    const float remaining = target_distance_ - projected_distance();
    if (remaining <= 0.0f) {
        motion_.stop();
        return true;
    }

    if (state_frames_ > kWalkTimeoutFrames) {
        motion_.stop();
        std::fprintf(stderr, "[Stage5Real] walk timeout in state %d\n",
            static_cast<int>(state_));
        return true;
    }

    const float yaw_error = norm_yaw(target_yaw_ - sensor_.abs_yaw);
    const float yaw_cmd = clamp(1.10f * yaw_error, -0.28f, 0.28f);
    // 第一段是水平桥，不使用斜桥的降速补偿；第二至第五段才降速。
    const float nominal_speed = state_ == State::WALK_SECTION_1
        ? kFlatSpeed : kSlopeSpeed;
    const float speed = remaining < 0.30f ? std::min(nominal_speed, 0.08f) : nominal_speed;
    const float lateral_speed = state_ == State::WALK_SECTION_1
        ? 0.0f : kSlopeLateralSpeed;
    motion_.set_walk_velocity_step(speed, lateral_speed, yaw_cmd, kStepHeight);
    return false;
}

void Stage5Real::begin_turn(State next_state, float yaw_offset) {
    next_state_ = next_state;
    target_yaw_ = norm_yaw(start_yaw_ + yaw_offset);
    stable_frames_ = 0;
    state_frames_ = 0;

    if (next_state == State::JUMP_PREPARE) state_ = State::TURN_TO_DROP;
    else if (next_state == State::WALK_SECTION_2) state_ = State::TURN_LEFT;
    else if (next_state == State::WALK_SECTION_3) state_ = State::TURN_RIGHT_1;
    else if (next_state == State::WALK_SECTION_4) state_ = State::TURN_RIGHT_2;
    else state_ = State::TURN_RIGHT_3;
}

bool Stage5Real::update_turn() {
    const float yaw_error = norm_yaw(target_yaw_ - sensor_.abs_yaw);
    if (std::abs(yaw_error) < kYawTolerance) {
        ++stable_frames_;
        motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, kStepHeight);
        if (stable_frames_ >= kTurnStableFrames) return true;
    } else {
        stable_frames_ = 0;
        const float yaw_cmd = clamp(1.0f * yaw_error, -0.38f, 0.38f);
        motion_.set_walk_velocity_step(0.0f, 0.0f, yaw_cmd, kStepHeight);
    }

    if (state_frames_ > kTurnTimeoutFrames) {
        motion_.stop();
        std::fprintf(stderr, "[Stage5Real] turn timeout in state %d\n",
            static_cast<int>(state_));
        return true;
    }
    return false;
}

float Stage5Real::projected_distance() const {
    const float dx = sensor_.odom_x - segment_start_x_;
    const float dy = sensor_.odom_y - segment_start_y_;
    return dx * std::cos(target_yaw_) + dy * std::sin(target_yaw_);
}
