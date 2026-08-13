#include "stage3_real_test.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage3RealTest — 测试版: 伙伴连通域寻线算法实验 (2026-08-14)
// 视觉: LaneDetector v2 (test/real, 连通域轨迹跟踪 + lookahead 采样)
//   ⚠ 伙伴版 curvature 是无符号方差 → 本测试版不做曲率前馈
// 控制: yaw = clamp(-KP*off - 微分限幅, ±YAW_LIM), 丢线保持转向衰减
// 低头: 破限(x_effect_scale_pos=+30) + 303 rpy_des[1]=pitch (test17已验证)
// 编译: colcon build --cmake-args -DUSE_TEST_REAL_STAGE3=ON
// 日志: [S3T] 前缀, 与正式版 [S3S] 区分
// ═══════════════════════════════════════════════════════════

namespace {
constexpr float WALK_V   = 0.30f;    // 巡线前进速度 m/s
constexpr float PITCH    = 0.32f;    // 低头 0.32 rad (~18°)
constexpr float KP_VIS   = 1.0f;     // 视觉巡线增益 (offset→yaw)
constexpr float KD_VIS   = 0.15f;    // 微分预测增益
constexpr float KD_LIM   = 0.25f;    // 微分项单独限幅 (防检测噪声尖峰)
constexpr float YAW_LIM  = 0.8f;     // 巡线 yaw 限幅
constexpr float FWD_DIST = 3.0f;     // 巡线总距离 m
constexpr double SCALE_HACK    = 30.0;   // 破限: x_effect_scale_pos=+30
constexpr double SCALE_RESTORE = -0.55;  // 复原默认值
constexpr bool   TEST_HOLD = false; // true=201原地低头测试 false=正式巡线
}  // namespace

void Stage3RealTest::init() {
    done_      = false;
    loc_ready_ = false;
    pitch_hold_ = 0;
    traveled_   = 0.0f;
    phase_     = Phase::WAIT_READY;

    // ── 站起 (与 Stage1 同款已验证序列) ──
    motion_.locomotion();
    bool svc_ready = motion_.wait_motion_result_ready(5);
    motion_.stand();
    rclcpp::sleep_for(std::chrono::seconds(3));
#ifdef DEBUG_SENSOR
    fprintf(stderr, "[S3T] 站起: 服务%s odom=(%.2f,%.2f) absYaw=%.2f\n",
            svc_ready ? "✅就绪" : "❌超时", sensor_.odom_x, sensor_.odom_y, sensor_.abs_yaw);
    fflush(stderr);
#endif
    RCLCPP_INFO(rclcpp::get_logger("stage3_real_test"),
                "[Stage3RealTest] init: 伙伴连通域寻线实验 (低头%.2frad 速度%.2fm/s 巡线%.1fm)",
                PITCH, WALK_V, FWD_DIST);
}

void Stage3RealTest::run() {
    if (done_) return;

    // ── ① 等定位就绪 (init 在 spin 前, 回调没跑 absYaw 恒0, 同Stage1) ──
    if (phase_ == Phase::WAIT_READY) {
        if (sensor_.abs_yaw != 0.0f || sensor_.odom_x != 0.0f) {
            phase_    = Phase::LANE_FOLLOW;
            last_x_   = sensor_.odom_x;
            last_y_   = sensor_.odom_y;
            traveled_ = 0.0f;
            if (!TEST_HOLD) {
                motion_.set_user_param_double_lcm("x_effect_scale_pos", SCALE_HACK);  // 破限(前进中生效)
            }
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3T] 定位就绪 odom=(%.2f,%.2f), %s\n",
                    sensor_.odom_x, sensor_.odom_y,
                    TEST_HOLD ? "201原地低头" : "破限低头巡线开始(伙伴算法)");
            fflush(stderr);
#endif
        } else {
#ifdef DEBUG_SENSOR
            static int w_ = 0;
            if (++w_ % 50 == 0) {
                fprintf(stderr, "[S3T] 等待定位... absYaw=%.2f\n", sensor_.abs_yaw);
                fflush(stderr);
            }
#endif
            return;   // 狗保持站立(111)静止等待
        }
    }

    // ── ② 低头巡线 / 原地测试 ──
    if (phase_ == Phase::LANE_FOLLOW) {
        if (TEST_HOLD) {
            // 201原地低头 (测试形态): 每3帧≈33Hz持续发布, 不发303
            if (++pitch_hold_ % 3 == 0)
                motion_.set_body_pitch(PITCH);
#ifdef DEBUG_SENSOR
            static int dbg_ = 0;
            if (++dbg_ % 10 == 0) {
                fprintf(stderr, "[S3T] pitch_map=%.1f° off=%.2f curv=%.2f valid=%d | 原地低头\n",
                        sensor_.pitch_map * 180.0f / M_PI, sensor_.lane_offset,
                        sensor_.lane_curvature, (int)sensor_.lane_valid);
                fflush(stderr);
            }
#endif
            return;
        }

        // ── 正式巡线: 破限低头前进 + 视觉回中 ──
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x;
        last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;
        traveled_ += moved;

        if (traveled_ >= FWD_DIST) {
            motion_.set_user_param_double_lcm("x_effect_scale_pos", SCALE_RESTORE);  // 复原
            motion_.stop();
            phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3T] 巡线%.1fm完成, 复原破限, DONE\n", traveled_);
            fflush(stderr);
#endif
            return;
        }

        // 视觉回中 + 微分预测 + 丢线保持 (无曲率前馈: 伙伴curvature无符号)
        float yaw_cmd = 0.0f;
        float d_off = 0.0f;
        if (sensor_.lane_valid) {
            float off = sensor_.lane_offset;
            d_off = (off - last_offset_) * 100.0f;   // 100Hz差分≈变化速率
            last_offset_ = off;
            float kd_term = std::max(-KD_LIM, std::min(KD_LIM, KD_VIS * d_off));
            yaw_cmd = std::max(-YAW_LIM, std::min(YAW_LIM, -KP_VIS * off - kd_term));
            last_yaw_ = yaw_cmd;
        } else {
            // 丢线: 保持最后转向并逐帧衰减(~1.5s衰减完), 把赛道拉回画面
            last_yaw_ *= 0.85f;
            if (std::abs(last_yaw_) < 0.03f) last_yaw_ = 0.0f;
            yaw_cmd = last_yaw_;
            last_offset_ = 0.0f;
        }
        motion_.set_walk_velocity_pitch(WALK_V, 0.0f, yaw_cmd, PITCH);
#ifdef DEBUG_SENSOR
        static int dbg2_ = 0;
        if (++dbg2_ % 10 == 0) {
            fprintf(stderr, "[S3T] %.1f/%.1fm pitch_map=%.1f° off=%.2f curv=%.2f d=%.2f valid=%d yaw=%.2f\n",
                    traveled_, FWD_DIST, sensor_.pitch_map * 180.0f / M_PI,
                    sensor_.lane_offset, sensor_.lane_curvature, d_off,
                    (int)sensor_.lane_valid, yaw_cmd);
            fflush(stderr);
        }
#endif
        return;
    }

    if (phase_ == Phase::DONE) done_ = true;
}

