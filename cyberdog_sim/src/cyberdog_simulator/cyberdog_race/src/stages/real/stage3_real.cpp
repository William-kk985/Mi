#include "cyberdog_race/stages/real/stage3_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage3Real 真机版 — 第3赛段: 低头 + 视觉检测 (测试形态, 不前进)
// 低头: 201姿态控制 set_body_pitch(0.25) 原地保持 → ~14° (无步态夹持, 无需破限)
//   ⚠ 201 servo 需持续发布(停4帧=Servo data lost) → 控制循环100Hz每3帧发一次≈33Hz
//   ⚠ 不能与 303 踏步交替(模式打架, test17教训) → 只发201, 狗静止
// 视觉: LaneDetector(on_rgb /image_rgb) 持续检测, Web 标注看两侧黄线
// ⚠ 测试形态: 原地低头不动, 便于调视觉; 正式巡线待验证 (2026-08-12)
// 真机约定: 正值=低头 (2026-08-08 舵机方向确认)
// ═══════════════════════════════════════════════════════════

namespace {
constexpr float PITCH        = 0.25f;    // 低头 0.25 rad (201姿态, 原地可到~14°)
constexpr int   PITCH_EVERY  = 3;       // 100Hz循环每3帧发一次 ≈33Hz (servo需20Hz)
}  // namespace

void Stage3Real::init() {
    done_      = false;
    loc_ready_ = false;
    pitch_hold_ = 0;
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
                "[Stage3Real] init: 低头+视觉 测试形态(201原地低头不动)");
}

void Stage3Real::run() {
    if (done_) return;

    // ── ① 等定位就绪再开始 (init 在 spin 前, 回调没跑 absYaw 恒0, 同Stage1) ──
    // ⚠ 等定位期间不发任何303命令(111站立保持), 否则303步态残留会干扰后面的201低头 (2026-08-12)
    if (phase_ == Phase::WAIT_READY) {
        if (sensor_.abs_yaw != 0.0f || sensor_.odom_x != 0.0f) {
            phase_ = Phase::LOW_HOLD;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S3Stage] 定位就绪 odom=(%.2f,%.2f), 201低头保持\n",
                    sensor_.odom_x, sensor_.odom_y);
            fflush(stderr);
#endif
        } else {
#ifdef DEBUG_SENSOR
            static int w_ = 0;
            if (++w_ % 50 == 0) {   // 静止等待, 不发命令
                fprintf(stderr, "[S3S] 等待定位... absYaw=%.2f\n", sensor_.abs_yaw);
                fflush(stderr);
            }
#endif
            return;   // 狗保持站立(111)静止等待, 不发303踏步
        }
    }

    // ── ② 201原地低头保持 + 视觉检测 (不前进) ──
    if (phase_ == Phase::LOW_HOLD) {
        // 201姿态低头: 持续发布保持(每3帧≈33Hz), 狗静止不动 (不混303防打架)
        if (++pitch_hold_ % PITCH_EVERY == 0)
            motion_.set_body_pitch(PITCH);
        // 视觉检测由 on_rgb 持续运行, 这里只打印状态供观察
#ifdef DEBUG_SENSOR
        static int dbg_ = 0;
        if (++dbg_ % 10 == 0) {
            fprintf(stderr, "[S3S] pitch_map=%.1f° off=%.2f curv=%.0f valid=%d | 原地低头\n",
                    sensor_.pitch_map * 180.0f / M_PI, sensor_.lane_offset,
                    sensor_.lane_curvature, (int)sensor_.lane_valid);
            fflush(stderr);
        }
#endif
        return;   // 测试形态: 不前进, 不结束 (保持原地低头+视觉)
    }
}

