#include "cyberdog_race/stages/real/stage1_real.hpp"
#include "cyberdog_race/debug_config.hpp"   // ★ 必须显式 include, 否则 DEBUG_MOTION/SENSOR/STAGE 宏不生效(打印没编译)
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage1Real 真机版 — 石径探路
// 步高 0.15 | 前进 6m | 视觉巡线 + 里程计 + IMU 转90°
// 真机接口: set_walk_velocity_step = 303 WALK_USERTROT + step_height
//   (motion_servo_cmd.step_height 字段 = 真机正确步高通道)
// ═══════════════════════════════════════════════════════════

namespace {

constexpr float STEP_H        = 0.17f;   // 步高 0.17 (0.20 会不稳, 取偏小)
constexpr float WALK_V        = 0.18f;   // 正常前进速度 m/s
constexpr float RUSH_V        = 0.40f;   // 卡住冲刺速度 (柔化防猛冲)
constexpr float BONUS_SWING   = 0.06f;   // 步幅增益 cmpc_bonus_swing (默认 0.05)
constexpr float GAIT_PERIOD   = 0.30f;   // 步态周期 gait_period_time (默认 0.5; 调小=步频高=小碎步更稳)
constexpr float GOAL_DIST     = 3.70f;   // 前进 3.70m
constexpr float LAT_COMP      = 0.02f;   // 开环向左打底 (Stage1 物理右偏, 正=左)
constexpr float TURN_YAW      = M_PI / 2.0f;  // 目标转角 90°
constexpr float TURN_SPEED    = 0.60f;   // 转向速度 rad/s (+=左转)
constexpr float TURN_DONE_ERR = 0.04f;  // 转向完成误差
constexpr float TURN_SLOW_ERR = 0.25f;  // 末期减速误差阈值, <此值半速防过冲
constexpr int   TURN_SETTLE_FRAMES = 15; // 转到位停稳 0.15s
constexpr float KP_YAW        = 0.8f;    // 视觉比例
constexpr float KD_YAW        = 0.3f;    // 视觉微分
constexpr float IMU_WEIGHT    = 0.3f;    // IMU 回正权重(视觉为主)
constexpr float STUCK_DIST    = 0.003f;  // 卡住判定位移阈值 m (小碎步每帧 0.3-0.7cm, 阈值太大误判假冲刺)
constexpr int   STUCK_THRESH  = 120;     // 卡住帧数 (~3s, 防小碎步误判)
constexpr int   RUSH_FRAMES   = 8;       // 冲刺帧数
constexpr int   LANE_LOST_LIM = 10;      // 丢线容忍帧数

}  // namespace

void Stage1Real::init() {
    phase_       = Phase::FORWARD;
    done_        = false;
    stuck_       = 0;
    rush_        = 0;
    lane_lost_   = 0;
    prev_offset_ = 0.0f;
    traveled_    = 0.0f;
    turn_guard_  = 0;
    turn_settle_ = 0;
    loc_ready_   = false;   // 定位待就绪: 构造函数在 spin 前, global_to_robot 恒0, 需在 run() 里等

    // ── 站起: 完全移植 behavior test 已验证动作序列 ──
    //   构造函数 Motion init 已做 recovery+locomotion; 这里与 run_test() 开头一致再切一次
    //   forward_test 同款: stand()(111官方站立) → 等站稳
    motion_.locomotion();                                    // 切行走模式(run_test 开头同款)
    bool svc_ready = motion_.wait_motion_result_ready(5);    // 等 MotionResultCmd 服务就绪
    motion_.stand();                                         // RECOVERYSTAND 官方站立
    rclcpp::sleep_for(std::chrono::seconds(3));              // 等真正站起(服务异步)
#ifdef DEBUG_SENSOR
    fprintf(stderr, "[S1S] 站起: 服务%s odom=(%.2f,%.2f) yaw=%.2f absYaw=%.2f tof=%.2f\n",
            svc_ready ? "✅就绪" : "❌超时", sensor_.odom_x, sensor_.odom_y, sensor_.yaw, sensor_.abs_yaw, sensor_.tof_clearance);
    fflush(stderr);
#endif

    motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);  // 预热原地踏步
    // ★ 真机步高正确通道: motion_servo_cmd.step_height 真机忽略!
    //   必须走 LCM robot_control_cmd(7671) set_step_height(起步前设一次)
    motion_.set_step_height(STEP_H, STEP_H);
    motion_.set_body_pitch(-0.10f);                          // pitch 正值=低头 → -0.10=轻微抬头

    // ── 步幅加大: bonus 同时作用于 x/y 落足点 (侧移步长也变远)
    // ⚠ 写一次即可 (RT板对动态写敏感; 重启自动回 yaml 默认0.05)
    motion_.set_user_param_double_lcm("cmpc_bonus_swing", BONUS_SWING);

    // ── 步频加快: 周期小→步幅小=小碎步更稳; 写一次即可
    motion_.set_user_param_double_lcm("gait_period_time", GAIT_PERIOD);

    RCLCPP_INFO(rclcpp::get_logger("stage1_real"), "[Stage1Real] init: 步高%.2f 步频%.2fHz 前进%.1fm",
                STEP_H, 1.0 / GAIT_PERIOD, GOAL_DIST);
}

void Stage1Real::run() {
    if (done_) return;

    // ── ② 最后原地转 90° (abs_yaw 闭环) ──
    //   ⚠ 反馈必须用 abs_yaw(global_to_robot.rpy[2]); IMU yaw 真机恒0 别用
    if (phase_ == Phase::TURN) {
        float yaw_err = norm_yaw(start_yaw_ + TURN_YAW - sensor_.abs_yaw);
        // 停稳重校正: 转到位先停 0.15s 等 abs_yaw 稳定, 停稳后误差还大→低速补转
        if (turn_settle_ > 0) {
            motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);
            if (++turn_settle_ <= TURN_SETTLE_FRAMES) return;
            turn_settle_ = 0;
            if (std::abs(yaw_err) < TURN_DONE_ERR) {
                motion_.stop();
                phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S1Stage] 转向完成 err=%.2f, DONE\n", yaw_err);
                fflush(stderr);
#endif
                return;
            }
            turn_guard_ = 0;                 // 还偏 → 低速补转
            return;
        }
        bool turn_done = (turn_guard_ > 20) && (std::abs(yaw_err) < TURN_DONE_ERR);  // 至少转0.2s防跳变
        turn_guard_++;
        if (turn_done) {                     // 转到位 → 停稳确认
            motion_.stop();
            turn_settle_ = 1;
        } else {
            // 末期减速: 误差<0.25rad 半速, 减少过冲
            float spd = (std::abs(yaw_err) < TURN_SLOW_ERR) ? TURN_SPEED * 0.5f : TURN_SPEED;
            motion_.set_walk_velocity_step(0.0f, 0.0f, yaw_err > 0 ? spd : -spd, STEP_H);
#ifdef DEBUG_STAGE
            if (turn_guard_ % 20 == 0) {
                fprintf(stderr, "[S1Stage] 转向中 err=%.2f absYaw=%.2f 目标=%.2f\n",
                        yaw_err, sensor_.abs_yaw, start_yaw_ + TURN_YAW);
                fflush(stderr);
            }
#endif
        }
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; return; }

    // ── ① 等定位就绪再开始前进 ──
    // ⚠ init 在构造函数(spin前)跑, global_to_robot 回调没执行 → absYaw 恒0;
    //   必须 run()(spin后)里等定位就绪再锁起点, 否则起点锁 0 → 回正疯狂转向
    if (!loc_ready_) {
        // ⚠ 右拐事故: abs_yaw 就绪但 yaw_odom 首包未到→起点锁 0→满幅右转; 必须等 odom_yaw_ready
        // ── 定位收敛判据: 位置连续 50 帧(0.5s)变化<5cm 才锁起点, 防 SLAM 初始化跳变 ──
        const float jx = sensor_.odom_x - loc_prev_x_;
        const float jy = sensor_.odom_y - loc_prev_y_;
        loc_prev_x_ = sensor_.odom_x; loc_prev_y_ = sensor_.odom_y;
        const bool sl_ready = (sensor_.abs_yaw != 0.0f || sensor_.odom_x != 0.0f) && sensor_.odom_yaw_ready;
        if (sl_ready) {
            if (std::hypot(jx, jy) < 0.05f) ++loc_stable_cnt_;
            else loc_stable_cnt_ = 0;
        }
        if (sl_ready && loc_stable_cnt_ >= 50) {
            loc_ready_  = true;
            start_x_    = sensor_.odom_x;
            start_y_    = sensor_.odom_y;
            start_yaw_      = sensor_.abs_yaw_lp;   // 低通值
            start_odom_yaw_ = sensor_.yaw_odom;
            start_imu_yaw_  = sensor_.yaw_imu;      // external_imu 相对锚
            lat_guard_prev_ = 0.0f;
            lat_guard_cnt_  = 0;
            lat_hold_       = false;
            step_time_  = 0;
            last_x_     = sensor_.odom_x;
            last_y_     = sensor_.odom_y;
            traveled_   = 0.0f;
            turn_guard_ = 0;
#ifdef DEBUG_SENSOR
            fprintf(stderr, "[S1S] 定位就绪: odom=(%.2f,%.2f) absYaw=%.2f 开始前进\n",
                    sensor_.odom_x, sensor_.odom_y, sensor_.abs_yaw);
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);  // 原地踏步等定位
#ifdef DEBUG_SENSOR
            static int wait_dbg_ = 0;
            if (++wait_dbg_ % 50 == 0) {
                fprintf(stderr, "[S1S] 等待定位... absYaw=%.2f\n", sensor_.abs_yaw);
                fflush(stderr);
            }
#endif
            return;
        }
    }

    // ── ① 前进 3.7m (累计位移判定, 带跳变保护) ──
    float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;
    if (moved > 0.25f) moved = 0.0f;   // 跳变保护: 单帧>25cm 视为定位跳变, 忽略
    traveled_ += moved;

    if (traveled_ >= GOAL_DIST) {
        motion_.stop();
        phase_ = Phase::TURN;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S1Stage] 前进 %.2fm 完成, 转 90°\n", traveled_);
        fflush(stderr);
#endif
        return;
    }
#ifdef DEBUG_SENSOR
    static int dbg_ = 0;
    if (++dbg_ % 10 == 0) {
        fprintf(stderr, "[S1S] odom=(%.2f,%.2f) yaw=%.2f absYaw=%.2f traveled=%.2f moved=%.4f tof=%.2f elev=%.2f stuck=%d rush=%d\n",
                sensor_.odom_x, sensor_.odom_y, sensor_.yaw, sensor_.abs_yaw, traveled_, moved,
                sensor_.tof_clearance, sensor_.tof_elev_max, stuck_, rush_);
        fflush(stderr);
    }
#endif

    if (rush_ > 0) {                                         // 冲刺中
        motion_.set_walk_velocity_step(RUSH_V, 0.0f, 0.0f, STEP_H);
        --rush_;
        return;
    }

    if (moved < STUCK_DIST) {
        if (++stuck_ >= STUCK_THRESH) {
            stuck_ = 0;
            rush_ = RUSH_FRAMES;
            RCLCPP_WARN(rclcpp::get_logger("stage1_real"), "[Stage1Real] 卡住! 冲刺脱困");
        }
    } else {
        stuck_ = 0;
    }

    // ── ① 直行回正抗抖: external_imu 相对角主反馈, 未就绪退化 abs_yaw_lp ──
    //   ⚠ external_imu 500Hz, 石径上比 VIO yaw(±8°抖)稳
    const float yaw_err = sensor_.imu_yaw_ready
        ? norm_yaw(sensor_.yaw_imu - start_imu_yaw_)
        : norm_yaw(sensor_.abs_yaw_lp - start_yaw_);
    float yaw_cmd = std::max(-0.5f, std::min(0.5f, -yaw_err * 0.8f));
    const float sy = std::sin(start_yaw_), cy = std::cos(start_yaw_);   // ⚠ 用 absYaw 起点
    const float lat = (sensor_.odom_x - start_x_) * (-sy) + (sensor_.odom_y - start_y_) * cy;   // SLAM位置相对起点线
    // ── SLAM 漂移保护: 横向纠正近满幅持续 1s 且 |latDev| 不降反升 → 冻结横向纠正 ──
    if (!lat_hold_) {
        const bool near_limit = std::abs(lat * 0.5f) >= 0.12f;   // vy近满幅
        const bool worsening  = std::abs(lat) > std::abs(lat_guard_prev_) + 0.005f;
        if (near_limit && worsening) ++lat_guard_cnt_;
        else lat_guard_cnt_ = 0;
        lat_guard_prev_ = lat;
        if (lat_guard_cnt_ >= 100) {   // 连续1秒
            lat_hold_ = true;
            RCLCPP_WARN(rclcpp::get_logger("stage1_real"), "[Stage1Real] SLAM漂移保护: 横向纠正冻结 (latDev=%.2fm 不降反升)", lat);
        }
    }
    const float vy_lock = lat_hold_ ? 0.0f : std::max(-0.15f, std::min(0.15f, -lat * 0.5f));
    // 叠加开环向左补偿: 闭环 vy_lock 拉不回物理右偏, 加 LAT_COMP 打底
    const float vy_cmd = lat_hold_ ? 0.0f : std::max(-0.15f, std::min(0.15f, vy_lock + LAT_COMP));
#ifdef DEBUG_MOTION
    static int dbg_m_ = 0;
    if (++dbg_m_ % 10 == 0) {
        fprintf(stderr, "[S1M] cmd v=(%.2f,%.2f,%.2f) yawErr=%.2f imuYaw=%.2f latDev=%.2f 已走=%.2f 步高%.2f\n",
                WALK_V, vy_cmd, yaw_cmd, yaw_err, sensor_.yaw_imu - start_imu_yaw_, lat, traveled_, STEP_H);
        fflush(stderr);
    }
#endif
    motion_.set_walk_velocity_step(WALK_V, vy_cmd, yaw_cmd, STEP_H);
}
