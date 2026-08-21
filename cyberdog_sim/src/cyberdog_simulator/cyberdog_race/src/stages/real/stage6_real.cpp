#include "cyberdog_race/stages/real/stage6_real.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
constexpr float kTurn48 = 48.0f * kDegToRad;
constexpr float kDiagonal41 = 41.0f * kDegToRad;
}  // namespace

void Stage6Real::init() {
    done_ = false;
    pose_initialized_ = false;
    lie_sent_ = false;
    state_frames_ = 0;
    stable_frames_ = 0;
    state_ = State::MOVE_FORWARD_1;
    motion_.stop();
    motion_.set_body_params_lcm(0.0f, 0.0f, kBodyHeight);
}

void Stage6Real::begin_walk(
    State state, float local_x, float local_y, float distance, float target_yaw)
{
    state_ = state;
    target_yaw_ = norm_yaw(target_yaw);
    segment_x_ = sensor_.odom_x;
    segment_y_ = sensor_.odom_y;
    segment_dir_x_ = local_x;
    segment_dir_y_ = local_y;
    segment_distance_ = distance;
    state_frames_ = 0;
}

bool Stage6Real::update_walk() {
    ++state_frames_;

    // 将机身局部速度方向转换到地图坐标，按投影计算当前段进度。
    // 真机 vel_des 约定：x 为前方，y 为左方；target_yaw_ 是地图朝向。
    const float world_dir_x = segment_dir_x_ * std::cos(target_yaw_)
                            - segment_dir_y_ * std::sin(target_yaw_);
    const float world_dir_y = segment_dir_x_ * std::sin(target_yaw_)
                            + segment_dir_y_ * std::cos(target_yaw_);
    const float dx = sensor_.odom_x - segment_x_;
    const float dy = sensor_.odom_y - segment_y_;
    const float progress = dx * world_dir_x + dy * world_dir_y;

    if (progress >= segment_distance_ - kDistanceTolerance) {
        motion_.stop();
        return true;
    }
    if (state_frames_ > kWalkTimeoutFrames) {
        motion_.stop();
        return false;
    }

    const float yaw_error = norm_yaw(target_yaw_ - sensor_.abs_yaw);
    const float yaw_cmd = std::max(-kYawMax, std::min(kYawMax, kYawKp * yaw_error));
    motion_.set_walk_velocity_step(kSpeed * segment_dir_x_,
                                   kSpeed * segment_dir_y_,
                                   yaw_cmd, kStepHeight);
    return false;
}

void Stage6Real::begin_turn(State state, float target_yaw) {
    state_ = state;
    target_yaw_ = norm_yaw(target_yaw);
    state_frames_ = 0;
    stable_frames_ = 0;
}

bool Stage6Real::update_turn() {
    ++state_frames_;
    const float yaw_error = norm_yaw(target_yaw_ - sensor_.abs_yaw);
    if (std::abs(yaw_error) <= kYawTolerance) {
        motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, kStepHeight);
        if (++stable_frames_ >= kTurnStableFrames) {
            motion_.stop();
            return true;
        }
    } else {
        stable_frames_ = 0;
        const float yaw_cmd = std::max(-kTurnYawMax,
            std::min(kTurnYawMax, kYawKp * yaw_error));
        motion_.set_walk_velocity_step(0.0f, 0.0f, yaw_cmd, kStepHeight);
    }
    if (state_frames_ > kTurnTimeoutFrames) motion_.stop();
    return false;
}

void Stage6Real::run() {
    if (done_) return;

    if (!pose_initialized_) {
        // (2026-08-21 兼容修复: SensorData 无 odom_valid 字段, 用 odom_yaw_ready(收到过odom首包))
        if (!sensor_.odom_yaw_ready) {
            motion_.stop();
            return;
        }
        start_yaw_ = sensor_.abs_yaw;
        pose_initialized_ = true;
        begin_walk(State::MOVE_FORWARD_1, 1.0f, 0.0f, 0.25f, start_yaw_);
        std::fprintf(stderr,
            "[Stage6Real] start: forward 0.25m, right 48deg, forward 3.00m, "
            "left 48deg, forward 0.25m, left-back 41deg 3.36m, back 0.50m\n");
    }

    switch (state_) {
    case State::MOVE_FORWARD_1:
        if (update_walk()) begin_turn(State::TURN_RIGHT_48, start_yaw_ - kTurn48);
        break;
    case State::TURN_RIGHT_48:
        if (update_turn()) {
            begin_walk(State::MOVE_FORWARD_2, 1.0f, 0.0f, 1.80f,
                       start_yaw_ - kTurn48);
        }
        break;
    case State::MOVE_FORWARD_2:
        if (update_walk()) begin_turn(State::TURN_LEFT_48, start_yaw_);
        break;
    case State::TURN_LEFT_48:
        if (update_turn()) {
            begin_walk(State::MOVE_FORWARD_3, 1.0f, 0.0f, 0.80f, start_yaw_);
        }
        break;
    case State::MOVE_FORWARD_3:
        if (update_walk()) {
            // 左后41度：以“正左”向后偏41度，身体继续保持正前朝向。
            // 在 x 前、y 左的机身坐标中，方向单位向量为
            // (-sin(41°), cos(41°))。
            begin_walk(State::MOVE_LEFT_BACK_41,
                       -std::sin(kDiagonal41), std::cos(kDiagonal41),
                       2.60f, start_yaw_);
        }
        break;
    case State::MOVE_LEFT_BACK_41:
        if (update_walk()) {
            begin_walk(State::MOVE_BACKWARD, -1.0f, 0.0f, 0.40f, start_yaw_);
        }
        break;
    case State::MOVE_BACKWARD:
        if (update_walk()) {
            state_ = State::LIE_DOWN;
            state_frames_ = 0;
        }
        break;
    case State::LIE_DOWN:
        motion_.stop();
        if (!lie_sent_) {
            motion_.lie_down();
            lie_sent_ = true;
        }
        // 接口调用后保持一个短暂状态，避免下一 timer tick 重复触发动作。
        if (++state_frames_ >= 20) {
            state_ = State::DONE;
            done_ = true;
            std::fprintf(stderr, "[Stage6Real] completed\n");
        }
        break;
    case State::DONE:
        done_ = true;
        break;
    }
}

bool Stage6Real::is_done() { return done_; }