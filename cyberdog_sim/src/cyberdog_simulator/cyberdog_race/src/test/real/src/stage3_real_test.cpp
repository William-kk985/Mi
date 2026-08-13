#include "stage3_real_test.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage3RealTest — 测试版: 伙伴连通域寻线算法 + 伙伴控制 (2026-08-14)
// 视觉: LaneDetector v2 (test/real, 连通域轨迹跟踪 + lookahead 采样)
// 控制: 伙伴版 — 降速巡线, 丢线低速搜索, 低通微分阻尼 (不乘固定100Hz)
//   WALK_V=0.26 窄视野降速给单边曲线识别留时间
//   LOST_V=0.10 短暂丢线低速搜索, 禁止盲目前冲
//   filtered_d_offset = 0.25*raw + 0.75*old (逐帧偏差变化低通)
//   丢线<20帧: 保持最后转向慢衰减; ≥20帧: 停前进原地缓慢搜索
// 日志: [S3T] RCLCPP_INFO 落盘, ssh 可直接抓取分析
// ═══════════════════════════════════════════════════════════

namespace {
constexpr float WALK_V   = 0.26f;    // 实拍窄视野下适当降速，给单边曲线识别留出修正时间
constexpr float LOST_V   = 0.10f;    // 短暂丢线时低速搜索，禁止盲目前冲
constexpr float PITCH    = 0.32f;    // 低头 0.32 rad (~18°)
constexpr float KP_VIS   = 0.85f;
constexpr float KD_VIS   = 0.08f;    // 对滤波后的逐帧偏差变化做阻尼，不再乘固定100Hz
constexpr float YAW_LIM  = 0.65f;
constexpr float FWD_DIST = 3.0f;     // 巡线总距离 m
constexpr double SCALE_HACK    = 30.0;   // 破限: x_effect_scale_pos=+30 (前进中放大pitch限位)
constexpr double SCALE_RESTORE = -0.55;  // 复原默认值
constexpr bool   TEST_HOLD = true;  // true=201原地低头不动(调视觉) false=正式巡线 (2026-08-14 切原地)
constexpr int    LOST_SEARCH_TICKS = 20;  // 丢线多少控制帧(100Hz=0.2s)后停前进原地搜索
}  // namespace

void Stage3RealTest::init() {
    done_      = false;
    loc_ready_ = false;
    pitch_hold_ = 0;
    traveled_   = 0.0f;
    last_offset_ = 0.0f;
    last_yaw_ = 0.0f;
    filtered_d_offset_ = 0.0f;
    lost_frames_ = 0;
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
                "[Stage3RealTest] init: 伙伴视觉+伙伴控制 (低头%.2frad 速度%.2fm/s 巡线%.1fm)",
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
                fprintf(stderr, "[S3T] pitch_map=%.1f° off=%.2f valid=%d | 原地低头\n",
                        sensor_.pitch_map * 180.0f / M_PI, sensor_.lane_offset,
                        (int)sensor_.lane_valid);
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

        // ── 伙伴控制: 降速巡线 + 丢线减速搜索 + 低通微分 ──
        float yaw_cmd = 0.0f;
        float forward_v = WALK_V;
        if (sensor_.lane_valid) {
            float off = sensor_.lane_offset;
            const float raw_delta = off - last_offset_;
            filtered_d_offset_ = 0.25f * raw_delta + 0.75f * filtered_d_offset_;
            last_offset_ = off;
            yaw_cmd = std::max(-YAW_LIM, std::min(YAW_LIM,
                                -KP_VIS * off - KD_VIS * filtered_d_offset_));
            last_yaw_ = yaw_cmd;
            lost_frames_ = 0;
        } else {
            ++lost_frames_;
            forward_v = LOST_V;
            // 短暂丢线保持最后修正方向；长时间丢线后停止前进并缓慢原地搜索。
            if (lost_frames_ < LOST_SEARCH_TICKS) {
                last_yaw_ *= 0.96f;
            } else {
                forward_v = 0.0f;
                if (std::abs(last_yaw_) < 0.12f)
                    last_yaw_ = last_offset_ >= 0.0f ? -0.18f : 0.18f;
            }
            yaw_cmd = last_yaw_;
            filtered_d_offset_ *= 0.8f;
        }
        motion_.set_walk_velocity_pitch(forward_v, 0.0f, yaw_cmd, PITCH);
#ifdef DEBUG_SENSOR
        static int dbg2_ = 0;
        if (++dbg2_ % 10 == 0) {
            RCLCPP_INFO(rclcpp::get_logger("stage3_real_test"),
                "[S3T] %.1f/%.1fm pitch_map=%.1f° off=%.2f fd=%.3f lost=%d valid=%d yaw=%.2f v=%.2f",
                traveled_, FWD_DIST, sensor_.pitch_map * 180.0f / M_PI,
                sensor_.lane_offset, filtered_d_offset_, lost_frames_,
                (int)sensor_.lane_valid, yaw_cmd, forward_v);
        }
#endif
        return;
    }

    if (phase_ == Phase::DONE) done_ = true;
}

