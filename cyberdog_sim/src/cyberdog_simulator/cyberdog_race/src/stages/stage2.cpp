#include "cyberdog_race/stages/stage2.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// 绿色终端打印（受 DEBUG_STAGE 控制）
#ifdef DEBUG_STAGE
#define LOG_GREEN(msg) LOG_STAGE_GREEN("Stage2", msg)
#define LOG_GREENF(fmt, ...) LOG_STAGE_GREENF("Stage2", fmt, ##__VA_ARGS__)
#else
#define LOG_GREEN(msg)
#define LOG_GREENF(fmt, ...)
#endif
static float norm_yaw(float y) {
    while (y >  M_PI) y -= 2.f * M_PI;
    while (y < -M_PI) y += 2.f * M_PI;
    return y;
}

void Stage2::init() {
    done_      = false;
    wp_idx_    = 0;
    state_     = State::MOVE_TO_POINT;
    scan_found_ = false;
    scan_done_  = false;

    // ── 路径点定义 ──────────────────────────────────────────
    // yaw: -π/2=向上(y+), 0=向右(x+), π=向左(x-), π/2=向下(y-)
    // scan=true 表示到达后左右扫描±40°找橙色球
    //
    //  入口进来先向上走到 y=1.1，再向右到 (2.7,1.1) 扫描
    //  然后向左到 (0,1.1) 扫描，向上到 y=1.7 ...
    //  最后到 (2.7,4.2) 向左走到出口 (-0.15,4.2)

    float L = 0.f, R = 2.7f;
    // 暂时只测试走到 y=1.0
    waypoints_[0]  = {2.55f, 1.0f,   M_PI/2.f,     true};
    waypoints_[1]  = {0.2f,  1.0f,   M_PI/2.f,     true};
    waypoints_[2]  = {0.2f,  1.7f,   M_PI/2.f,     true};
    waypoints_[3]  = {2.7f,  1.7f,   M_PI/2.f,     true};
    waypoints_[4]  = {2.7f,  2.6f,   M_PI/2.f,     true};
    waypoints_[5]  = {0.2f,  2.6f,   M_PI/2.f,     true};
    waypoints_[6]  = {0.2f,  3.5f,   M_PI/2.f,     true};
    waypoints_[7]  = {2.7f,  3.5f,   M_PI/2.f,     true};
    waypoints_[8]  = {2.7f,  4.2f,   M_PI/2.f,     false};
    waypoints_[9]  = {-0.2f, 4.2f,   M_PI/2.f,     false};
    // 其余路径点暂时注释
    /*
    waypoints_[1]  = {L,     1.1f,  M_PI,         true };
    waypoints_[2]  = {L,     1.7f,  M_PI/2.f,     false};
    waypoints_[3]  = {R,     1.7f,  0.f,          true };
    waypoints_[4]  = {L,     1.7f,  M_PI,         true };
    waypoints_[5]  = {L,     2.6f,  M_PI/2.f,     false};
    waypoints_[6]  = {R,     2.6f,  0.f,          true };
    waypoints_[7]  = {L,     2.6f,  M_PI,         true };
    waypoints_[8]  = {L,     3.5f,  M_PI/2.f,     false};
    waypoints_[9]  = {R,     3.5f,  0.f,          true };
    waypoints_[10] = {L,     3.5f,  M_PI,         true };
    waypoints_[11] = {-0.15f,4.2f,  M_PI/2.f,     false};
    */

    // 第一步：只往 y 方向走，到达后转向 π（朝左）准备走向 wp[0]
    target_x_   = sensor_.odom_x;
    target_y_   = 1.0f;
    target_yaw_ = M_PI;

    motion_.locomotion();
#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage2"), "Stage2 init, entry x=%.2f yaw=%.3f", sensor_.odom_x, sensor_.yaw);
#endif
}

void Stage2::run() {
    if (done_) return;

    switch (state_) {

    case State::MOVE_TO_POINT:
        if (reached_pos(target_x_, target_y_)) {
            motion_.stop();
            state_ = State::TURN_TO_YAW;
            LOG_GREENF("✓ 到达位置 (%.2f, %.2f)", target_x_, target_y_);
        } else {
            // 第一步（wp_idx_==0）只判断 y，不管 x
            if (wp_idx_ == 0 && std::abs(sensor_.odom_y - target_y_) < POS_THRESH) {
                motion_.stop();
                state_ = State::TURN_TO_YAW;
                LOG_GREENF("✓ 到达 y=%.2f（忽略x）", target_y_);
            } else {
                navigate_to(target_x_, target_y_);
            }
        }
        break;

    case State::TURN_TO_YAW:
        if (reached_yaw(target_yaw_)) {
            motion_.stop();
            bool do_scan = (wp_idx_ > 0) && (wp_idx_ <= NUM_WP) && waypoints_[wp_idx_-1].scan && !scan_done_;
            if (do_scan) {
                scan_start_yaw_ = sensor_.yaw;
                scan_found_     = false;
                scan_done_      = false;
                scan_dir_       = 1;
                state_          = State::SCAN_LEFT;
                LOG_GREENF("✓ 转向完成 yaw=%.2f，开始扫描", sensor_.yaw);
            } else {
                LOG_GREENF("✓ 转向完成 yaw=%.2f", sensor_.yaw);
                scan_done_ = false;
                next_waypoint();
            }
        } else {
            turn_to(target_yaw_);
        }
        break;

    case State::SCAN_LEFT: {
        float yaw_diff = norm_yaw(sensor_.yaw - scan_start_yaw_);
        if (yaw_diff < SCAN_ANGLE) {
            motion_.set_velocity(0.f, 0.f, -TURN_SPEED);
        } else {
            motion_.stop();
            LOG_GREENF("→ 左扫到位 yaw=%.2f，识别中...", sensor_.yaw);
            state_ = State::SCAN_LEFT_CHECK;
        }
        break;
    }

    case State::SCAN_LEFT_CHECK: {
        if (sensor_.ball_found && sensor_.ball_dist < BALL_DIST_THRESH && ball_in_arena()) {
            scan_found_ = true;
            start_x_ = sensor_.odom_x;
            start_y_ = sensor_.odom_y;
            state_   = State::HIT_BALL;
            LOG_GREENF("✓ 左扫发现橙球！dist=%.2f", sensor_.ball_dist);
        } else {
            LOG_GREEN("→ 左扫未发现球，转回中心");
            state_ = State::SCAN_LEFT_RETURN;
        }
        break;
    }

    case State::SCAN_LEFT_RETURN: {
        float err = norm_yaw(scan_start_yaw_ - sensor_.yaw);
        if (std::abs(err) < YAW_THRESH) {
            motion_.stop();
            LOG_GREEN("✓ 左扫结束，开始右扫");
            state_ = State::SCAN_RIGHT;
        } else {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, err > 0 ? -cmd : cmd);
        }
        break;
    }

    case State::SCAN_RIGHT: {
        float yaw_diff = norm_yaw(sensor_.yaw - scan_start_yaw_);
        if (yaw_diff > -SCAN_ANGLE) {
            motion_.set_velocity(0.f, 0.f, TURN_SPEED);
        } else {
            motion_.stop();
            LOG_GREENF("→ 右扫到位 yaw=%.2f，识别中...", sensor_.yaw);
            state_ = State::SCAN_RIGHT_CHECK;
        }
        break;
    }

    case State::SCAN_RIGHT_CHECK: {
        if (sensor_.ball_found && sensor_.ball_dist < BALL_DIST_THRESH && ball_in_arena()) {
            scan_found_ = true;
            start_x_ = sensor_.odom_x;
            start_y_ = sensor_.odom_y;
            state_   = State::HIT_BALL;
            LOG_GREENF("✓ 右扫发现橙球！dist=%.2f", sensor_.ball_dist);
        } else {
            LOG_GREEN("→ 右扫未发现球，转回中心");
            state_ = State::SCAN_RIGHT_RETURN;
        }
        break;
    }

    case State::SCAN_RIGHT_RETURN: {
        float err = norm_yaw(scan_start_yaw_ - sensor_.yaw);
        if (std::abs(err) < YAW_THRESH) {
            motion_.stop();
            LOG_GREEN("✓ 右扫结束，扫描完成");
            scan_done_ = true;
            if (wp_idx_ < 10) {
                // 还有下一个路径点，取其朝向先转好再走
                target_yaw_ = waypoints_[wp_idx_].yaw;
                state_ = State::TURN_TO_YAW;
                LOG_GREENF("→ 准备转向下一点 yaw=%.2f", target_yaw_);
            } else {
                // 没有下一个点，转向 x 正方向（yaw=0）再结束
                target_yaw_ = 0.f;
                state_ = State::TURN_TO_YAW;
                LOG_GREEN("→ 扫描全部完成，转向 x 正方向");
            }
        } else {
            float cmd = std::max(0.1f, std::min(0.4f, std::abs(err) * 0.6f));
            motion_.set_velocity(0.f, 0.f, err > 0 ? -cmd : cmd);
        }
        break;
    }

    case State::HIT_BALL:
        if (sensor_.ball_found && sensor_.ball_dist > 0.15f) {
            float yaw_cmd = -0.5f * sensor_.ball_x;
            motion_.set_velocity(HIT_SPEED, 0.f, yaw_cmd);
#ifdef DEBUG_MOTION
            RCLCPP_INFO(rclcpp::get_logger("stage2"), "Hitting ball dist=%.2f", sensor_.ball_dist);
#endif
        } else {
            motion_.stop();
            state_ = State::BACK_TO_PATH;
            LOG_GREENF("✓ 撞球完成，退回 (%.2f,%.2f)", start_x_, start_y_);
        }
        break;

    case State::BACK_TO_PATH:
        if (reached_pos(start_x_, start_y_)) {
            motion_.stop();
            LOG_GREEN("✓ 退回路径点完成");
            next_waypoint();
        } else {
            navigate_to(start_x_, start_y_);
        }
        break;

    case State::DONE:
        done_ = true;
        motion_.stop();
        break;
    }
}

bool Stage2::is_done() { return done_; }

// ── 辅助函数 ────────────────────────────────────────────────

void Stage2::navigate_to(float tx, float ty) {
    float dx = tx - sensor_.odom_x;
    float dy = ty - sensor_.odom_y;
    float dist = std::sqrt(dx*dx + dy*dy);

    // 第一步（wp_idx_==0）：只往 y 方向直走，不转向
    if (wp_idx_ == 0) {
        float speed = std::min(MOVE_SPEED, std::abs(dy));
        motion_.set_velocity(speed, 0.f, 0.f);
        return;
    }

    // 球阵坐标（禁区中心）
    static const float BALL_X[4] = {-0.4f, 0.8f, 2.0f, 3.2f};
    static const float BALL_Y[4] = {1.3f, 2.1f, 2.9f, 3.7f};
    static constexpr float DANGER_R = 0.1f;  // 禁区半径

    // 检查当前位置是否接近任何球位
    float min_ball_dist = 99.f;
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            float bdx = sensor_.odom_x - BALL_X[c];
            float bdy = sensor_.odom_y - BALL_Y[r];
            float bd = std::sqrt(bdx*bdx + bdy*bdy);
            if (bd < min_ball_dist) min_ball_dist = bd;
        }
    }

    // 接近禁区时减速
    float speed_limit = MOVE_SPEED;
    if (min_ball_dist < DANGER_R) {
        speed_limit = MOVE_SPEED * (min_ball_dist / DANGER_R) * 0.5f;
        speed_limit = std::max(0.05f, speed_limit);
#ifdef DEBUG_MOTION
        RCLCPP_INFO(rclcpp::get_logger("stage2"), "Near ball! dist=%.2f speed_limit=%.2f",
                    min_ball_dist, speed_limit);
#endif
    }

    float desired_yaw = std::atan2(dy, dx);
    float yaw_err = norm_yaw(desired_yaw - sensor_.yaw);

#ifdef DEBUG_MOTION
    RCLCPP_INFO(rclcpp::get_logger("stage2"), "nav: pos=(%.2f,%.2f) tgt=(%.2f,%.2f) des_yaw=%.2f cur_yaw=%.2f err=%.2f",
                sensor_.odom_x, sensor_.odom_y, tx, ty, desired_yaw, sensor_.yaw, yaw_err);
#endif

    if (std::abs(yaw_err) > 0.35f) {
        float cmd = std::max(0.05f, std::min(0.4f, std::abs(yaw_err) * 0.6f));
        motion_.set_velocity(0.f, 0.f, yaw_err > 0 ? -cmd : cmd);
    } else {
        float speed = std::min(speed_limit, dist);
        float yaw_cmd = std::max(-0.5f, std::min(0.5f, -yaw_err * 2.0f));
        motion_.set_velocity(speed, 0.f, yaw_cmd);
    }
}

void Stage2::turn_to(float target_yaw) {
    float err = norm_yaw(target_yaw - sensor_.yaw);
    float cmd = std::max(0.1f, std::min(0.4f, std::abs(err) * 0.6f));
#ifdef DEBUG_MOTION
    RCLCPP_INFO(rclcpp::get_logger("stage2"), "turn_to: cur=%.2f target=%.2f err=%.2f cmd=%.2f",
                sensor_.yaw, target_yaw, err, err > 0 ? cmd : -cmd);
#endif
    motion_.set_velocity(0.f, 0.f, err > 0 ? -cmd : cmd);
}

bool Stage2::reached_pos(float tx, float ty) {
    float dx = tx - sensor_.odom_x;
    float dy = ty - sensor_.odom_y;
    return std::sqrt(dx*dx + dy*dy) < POS_THRESH;
}

bool Stage2::reached_yaw(float target_yaw) {
    return std::abs(norm_yaw(target_yaw - sensor_.yaw)) < YAW_THRESH;
}
void Stage2::next_waypoint() {
    if (wp_idx_ >= 10) {
        state_ = State::DONE;
        LOG_GREEN("✓ 所有路径点完成，赛段2结束");
        return;
    }
    auto& wp   = waypoints_[wp_idx_++];
    target_x_  = wp.x;
    target_y_  = wp.y;
    target_yaw_ = wp.yaw;
    state_     = State::MOVE_TO_POINT;
    LOG_GREENF("→ 走向 WP[%d] (%.2f, %.2f)", wp_idx_-1, wp.x, wp.y);
#ifdef DEBUG_STAGE
    RCLCPP_INFO(rclcpp::get_logger("stage2"), "Next WP[%d] (%.2f,%.2f) yaw=%.2f scan=%d",
                wp_idx_-1, wp.x, wp.y, wp.yaw, wp.scan);
#endif
}

bool Stage2::ball_in_arena() {
    // 根据当前位置和球的方向估算球的世界坐标
    // ball_x 是归一化图像坐标[-1,1]，ball_dist 是估算距离
    // 用当前 yaw + ball_x 偏角估算球的方向
    float ball_angle = sensor_.yaw + sensor_.ball_x * 0.5f;  // 粗略估算
    float ball_wx = sensor_.odom_x + sensor_.ball_dist * std::cos(ball_angle);
    float ball_wy = sensor_.odom_y + sensor_.ball_dist * std::sin(ball_angle);

    // 球阵范围：x[-0.4, 3.2]，y[1.3, 3.7]，留0.3m余量
    bool in_x = ball_wx > -0.7f && ball_wx < 3.5f;
    bool in_y = ball_wy > 1.0f  && ball_wy < 4.0f;

#ifdef DEBUG_SENSOR
    RCLCPP_INFO(rclcpp::get_logger("stage2"), "ball_in_arena: wx=%.2f wy=%.2f in=%d",
                ball_wx, ball_wy, in_x && in_y);
#endif
    return in_x && in_y;
}
