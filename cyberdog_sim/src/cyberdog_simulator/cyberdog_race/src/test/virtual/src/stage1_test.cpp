#include "cyberdog_race/stages/virtual/stage1.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage1 测试版 — 基于正式版，增加：
//   · 每帧状态打印
//   · LLM 实验性辅助决策
//   · 卡住时尝试强制跳跃脱困
// ═══════════════════════════════════════════════════════════

void Stage1::init() {
    done_             = false;
    at_junction_      = false;
    prev_offset_      = 0.0f;
    last_odom_x_      = sensor_.odom_x;
    last_odom_y_      = sensor_.odom_y;
    stuck_frames_     = 0;
    escape_frames_    = 0;
    lane_lost_frames_ = 0;
    motion_.locomotion();
    motion_.set_pitch(-0.26f);
    motion_.set_step_height(0.35f, 0.35f);

    fprintf(stderr, "\033[1;35m[TEST Stage1] init (test version)\033[0m\n");
}

void Stage1::run() {
    if (done_) return;

    // ── 测试版：每帧状态打印 ──
    fprintf(stderr, "[TEST Stage1] x=%.2f y=%.2f yaw=%.2f lane=%d off=%.2f\n",
            sensor_.odom_x, sensor_.odom_y, sensor_.yaw,
            sensor_.lane_valid, sensor_.lane_offset);

    // ── 测试版：卡住时尝试跳跃脱困 ──
    float dx = sensor_.odom_x - last_odom_x_;
    float dy = sensor_.odom_y - last_odom_y_;
    float dist = std::sqrt(dx*dx + dy*dy);
    last_odom_x_ = sensor_.odom_x;
    last_odom_y_ = sensor_.odom_y;

    if (dist < STUCK_DIST) {
        stuck_frames_++;
    } else {
        stuck_frames_ = 0;
    }

    if (stuck_frames_ > STUCK_THRESH && !at_junction_) {
        fprintf(stderr, "\033[1;31m[TEST Stage1] 卡住！尝试跳跃脱困\033[0m\n");
        motion_.jump();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        stuck_frames_ = 0;
    }

    // ── 路口原地转向阶段 ──────────────────────────────────────
    if (at_junction_) {
        constexpr float TARGET_YAW = M_PI / 2.0f;
        float yaw_err = norm_yaw(TARGET_YAW - sensor_.yaw);

        if (std::abs(yaw_err) < 0.05f) {
            done_ = true;
            motion_.set_pitch(0.0f);
            motion_.locomotion();
            motion_.stop();
            fprintf(stderr, "\033[1;34m[TEST Stage1] ✓ 转向完成 yaw=%.3f，第一赛段结束\033[0m\n", sensor_.yaw);
        } else {
            float turn = std::max(0.1f, std::min(0.5f, std::abs(yaw_err) * 0.6f));
            motion_.set_velocity(0.0f, 0.0f, yaw_err > 0 ? turn : -turn);
        }
        return;
    }

    // ── 正式逻辑：黄线循迹 + 弯道检测 + 路口识别 ──────────────
    // 与 stage1.cpp 保持一致的循迹逻辑
    if (sensor_.lane_valid) {
        float yaw_cmd = -sensor_.lane_offset * KP_YAW
                       - (sensor_.lane_offset - prev_offset_) * KD_YAW;
        yaw_cmd = std::max(-TURN_SPEED, std::min(TURN_SPEED, yaw_cmd));
        prev_offset_ = sensor_.lane_offset;

        if (sensor_.lane_curvature > CURVE_THRESH) {
            motion_.set_velocity(BASE_SPEED * 0.5f, 0.f, yaw_cmd);
        } else {
            motion_.set_velocity(BASE_SPEED, 0.f, yaw_cmd);
        }
        lane_lost_frames_ = 0;
    } else {
        lane_lost_frames_++;
        yaw_start_ = sensor_.yaw;  // 记录丢失时的yaw

        // 尝试原地旋转搜索黄线
        if (lane_lost_frames_ > 100) {
            float search_yaw = yaw_start_;
            float yaw_err = norm_yaw(search_yaw - sensor_.yaw);
            float turn = std::max(0.2f, 0.3f);
            motion_.set_velocity(0.0f, 0.0f, yaw_err > 0 ? turn : -turn);

            // 如果转了一圈还找不到→已经到路口了
            if (std::abs(yaw_err) < 0.05f &&
                sensor_.odom_x >= JUNCTION_X_MIN &&
                sensor_.odom_x <= JUNCTION_X_MAX) {
                at_junction_ = true;
                fprintf(stderr, "[TEST Stage1] 到达路口 at x=%.2f\n", sensor_.odom_x);
            }
        }
    }

    // ── 测试版：LLM 辅助决策占位 ──
    // TODO: 取消注释后联调 LLM
    // if (sensor_.lane_curvature > CURVE_THRESH) {
    //     std::string advice = llm_.ask("前方弯道，建议减速比例？");
    //     float ratio = 0.5f;
    //     if (!advice.empty()) {
    //         try { ratio = std::stof(advice); } catch (...) {}
    //         ratio = std::max(0.2f, std::min(1.0f, ratio));
    //     }
    //     motion_.set_velocity(BASE_SPEED * ratio, 0.f, yaw_cmd);
    // }
}

bool Stage1::is_done() { return done_; }
