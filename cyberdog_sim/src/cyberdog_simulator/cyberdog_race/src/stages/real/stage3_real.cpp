#include "cyberdog_race/stages/real/stage3_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage3Real 真机版 — 第3赛段: 低头 + 视觉巡线 (2026-08-13 正式形态)
// 低头: 破限(x_effect_scale_pos=+30) + 303 rpy_des[1]=pitch 前进低头 (test17已验证)
//   ⚠ 破限只在 vx>0 时生效 → 巡线持续前进; 201原地低头真机不生效(勿用)
// 巡线: lane_offset(>0=车偏左) → yaw_cmd=-KP*offset 回中; 丢线→直行
// 测试形态(TEST_HOLD=true): 201原地低头不动, 调视觉用
// 真机约定: 正值=低头 (2026-08-08 舵机方向确认)
// ═══════════════════════════════════════════════════════════

namespace {
constexpr float WALK_V   = 0.30f;    // 巡线前进速度 m/s
constexpr float PITCH    = 0.32f;    // 低头 0.32 rad (~18°) (2026-08-13: 0.38太低调回0.32)
constexpr float KP_VIS   = 1.0f;     // 视觉巡线增益 (offset→yaw) (2026-08-13: 0.5→1.0 转弯力度不够)
constexpr float KD_VIS   = 0.5f;     // 微分预测增益 (2026-08-13: 偏移趋势提前打方向, 冲出去也不至于太偏)
constexpr float YAW_LIM  = 0.8f;     // 巡线 yaw 限幅 (2026-08-13: 0.5→0.8)
constexpr float FWD_DIST = 3.0f;     // 巡线总距离 m (2026-08-13 待实测调整)
constexpr double SCALE_HACK    = 30.0;   // 破限: x_effect_scale_pos=+30 (前进中放大pitch限位)
constexpr double SCALE_RESTORE = -0.55;  // 复原默认值
constexpr bool   TEST_HOLD = false; // true=201原地低头测试 false=正式巡线
}  // namespace

void Stage3Real::init() {
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
    fprintf(stderr, "[S3S] 站起: 服务%s odom=(%.2f,%.2f) absYaw=%.2f\n",
            svc_ready ? "✅就绪" : "❌超时", sensor_.odom_x, sensor_.odom_y, sensor_.abs_yaw);
    fflush(stderr);
#endif
    RCLCPP_INFO(rclcpp::get_logger("stage3_real"),
                "[Stage3Real] init: %s (低头%.2frad 速度%.2fm/s 巡线%.1fm)",
                TEST_HOLD ? "201原地低头测试形态" : "破限低头前进+视觉巡线",
                PITCH, WALK_V, FWD_DIST);
}

void Stage3Real::run() {
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
            fprintf(stderr, "[S3Stage] 定位就绪 odom=(%.2f,%.2f), %s\n",
                    sensor_.odom_x, sensor_.odom_y,
                    TEST_HOLD ? "201原地低头" : "破限低头巡线开始");
            fflush(stderr);
#endif
        } else {
#ifdef DEBUG_SENSOR
            static int w_ = 0;
            if (++w_ % 50 == 0) {
                fprintf(stderr, "[S3S] 等待定位... absYaw=%.2f\n", sensor_.abs_yaw);
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
                fprintf(stderr, "[S3S] pitch_map=%.1f° off=%.2f curv=%.0f valid=%d | 原地低头\n",
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
            fprintf(stderr, "[S3Stage] 巡线%.1fm完成, 复原破限, DONE\n", traveled_);
            fflush(stderr);
#endif
            return;
        }

        // 视觉回中 + 微分预测 + 丢线保持 (2026-08-13): offset>0(车偏左)→右转(负yaw)
        //   微分项: 偏移在增大就提前加大转向, 没矫正过来冲出去也缓一缓
        //   丢线保持: 赛道出画面后按最后方向继续转(衰减), 把赛道拉回画面 (低头视角关键!)
        float yaw_cmd = 0.0f;
        if (sensor_.lane_valid) {
            float off = sensor_.lane_offset;
            float d_off = (off - last_offset_) * 100.0f;   // 100Hz差分≈变化速率
            last_offset_ = off;
            yaw_cmd = std::max(-YAW_LIM, std::min(YAW_LIM,
                                -KP_VIS * off - KD_VIS * d_off));
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
            fprintf(stderr, "[S3S] %.1f/%.1fm pitch_map=%.1f° off=%.2f valid=%d yaw=%.2f\n",
                    traveled_, FWD_DIST, sensor_.pitch_map * 180.0f / M_PI,
                    sensor_.lane_offset, (int)sensor_.lane_valid, yaw_cmd);
            fflush(stderr);
        }
#endif
        return;
    }

    if (phase_ == Phase::DONE) done_ = true;
}

