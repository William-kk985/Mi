#include "cyberdog_race/stages/stage2.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>

// 球阵参数（米）
static constexpr float COL_SPACING = 1.0f;
static constexpr float ROW_SPACING = 0.64f;
static constexpr float HIT_DIST    = 0.25f; // 撞击距离
static constexpr float MOVE_SPEED  = 0.2f;

void Stage2::init() {
    done_ = false;
    cur_row_ = 3;
    cur_col_ = 3;
    going_left_ = true;
    ball_done_.fill({false, false, false, false});
    ball_done_[3][3] = true;
    ball_done_[3][2] = true;
    ball_done_[2][3] = true;
    state_ = State::MOVING;
    motion_.locomotion();
#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage2"), "Stage2 init");
#endif
}

void Stage2::run() {
    if (done_) return;

    switch (state_) {
        case State::MOVING:
            // TODO: 里程计导航到下一个球位置
            // 到达后切换到DETECTING
            state_ = State::DETECTING;
            motion_.stop();
            break;

        case State::DETECTING:
            if (sensor_.ball_found) {
#ifdef DEBUG_SENSOR
                RCLCPP_INFO(rclcpp::get_logger("stage2"), "Ball found at [%d][%d] dist=%.2f x=%.2f",
                            cur_row_, cur_col_, sensor_.ball_dist, sensor_.ball_x);
#endif
                state_ = State::HITTING;
            } else {
                ball_done_[cur_row_][cur_col_] = true;
#ifdef DEBUG_STAGE
                RCLCPP_INFO(rclcpp::get_logger("stage2"), "No orange ball at [%d][%d], skip", cur_row_, cur_col_);
#endif
                state_ = State::MOVING;
            }
            break;

        case State::HITTING:
            if (sensor_.ball_dist > HIT_DIST) {
                float yaw = -0.5f * sensor_.ball_x;
                motion_.set_velocity(MOVE_SPEED, 0.0f, yaw);
#ifdef DEBUG_MOTION
                RCLCPP_INFO(rclcpp::get_logger("stage2"), "Hitting: dist=%.2f yaw=%.2f", sensor_.ball_dist, yaw);
#endif
            } else {
                ball_done_[cur_row_][cur_col_] = true;
                motion_.stop();
#ifdef DEBUG_STAGE
                RCLCPP_INFO(rclcpp::get_logger("stage2"), "Hit done [%d][%d]", cur_row_, cur_col_);
#endif
                state_ = State::MOVING;
            }
            break;

        case State::TO_EXIT:
            if (sensor_.lane_valid) {
                float yaw = -0.5f * sensor_.lane_offset;
                motion_.set_velocity(MOVE_SPEED, 0.0f, yaw);
#ifdef DEBUG_MOTION
                RCLCPP_INFO(rclcpp::get_logger("stage2"), "To exit: offset=%.2f", sensor_.lane_offset);
#endif
            } else {
                done_ = true;
                motion_.stop();
#ifdef DEBUG_STAGE
                RCLCPP_INFO(rclcpp::get_logger("stage2"), "Stage2 done");
#endif
            }
            break;
    }

    // 检查是否所有球处理完毕
    bool all_done = true;
    for (auto& row : ball_done_)
        for (auto v : row)
            if (!v) { all_done = false; break; }

    if (all_done && state_ != State::TO_EXIT) {
        state_ = State::TO_EXIT;
    }
}

bool Stage2::is_done() { return done_; }
