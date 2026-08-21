#include "cyberdog_race/stages/real/stage4_real.hpp"

#include "cyberdog_race/debug_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>

namespace {
inline float clamp01(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}
}  // namespace

float Stage4Real::clamp(float v, float lo, float hi) { return clamp01(v, lo, hi); }

// (2026-08-17 debug用) 状态名
const char* Stage4Real::state_name(State s) {
    switch (s) {
        case State::WAIT_FOR_SENSORS: return "WAIT_SENSORS";
        case State::FWD_1M:           return "FWD_1M";
        case State::TURN_R_IN:        return "TURN_R_IN";
        case State::LANE_OUT:         return "LANE_OUT";
        case State::PASS_LIMBAR:      return "PASS_LIMBAR";
        case State::RISE:             return "RISE";
        case State::KNOCK:            return "KNOCK";
        case State::SCAN_STOP:        return "SCAN_STOP";
        case State::TURN_BACK:        return "TURN_BACK";
        case State::LANE_BACK:        return "LANE_BACK";
        case State::TURN_R_OUT:       return "TURN_R_OUT";
        case State::TURN_L_FINAL:     return "TURN_L_FINAL";
        case State::FWD_FINAL:        return "FWD_FINAL";
        case State::TURN_END:         return "TURN_END";
        case State::RECOVERING:       return "RECOVERING";
        case State::DONE:             return "DONE";
    }
    return "?";
}

// ═══════════════════════════════════════════════════════════════
// init / 里程 / 感知
// ═══════════════════════════════════════════════════════════════
void Stage4Real::init() {
    state_ = State::WAIT_FOR_SENSORS;
    done_ = false;
    odom_initialized_ = false;

    // ── 站起 (2026-08-17 补上, 与Stage1/3同款: 没有站起狗趴着直接摔; 已站立跳过防“先摔再站”) ──
    if (sensor_.body_height < 0.25f) {   // 1234连跑时Stage3结束狗已站立, 不再重复站起
        motion_.locomotion();
        const bool svc_ready = motion_.wait_motion_result_ready(5);
        motion_.stand();
        rclcpp::sleep_for(std::chrono::seconds(3));
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] init: 站起 服务%s absYaw=%.2f\n",
                svc_ready ? "✅就绪" : "❌超时", sensor_.abs_yaw);
        fflush(stderr);
#endif
    } else {
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] init: 已站立 body_h=%.2f, 跳过站起\n", sensor_.body_height);
        fflush(stderr);
#endif
    }

    recovery_cmd_sent_ = false;
    gait_reengaged_ = false;
    recovery_attempts_ = 0;
    ready_frames_ = 0;
    state_frames_ = 0;
    sub_state_ = 0;
    round_count_ = 0;
    scan_done_out_ = false;   // (2026-08-18 回程停点已删, 只去程停)
    scan_frames_ = 0;
    entry_yaw_ = lane_yaw_ = back_yaw_ = 0.0f;
    target_yaw_ = 0.0f;
    sub_start_travel_ = 0.0f;
    knock_cx_ = 0.0f;
    ref_x_ = ref_y_ = 0.0f;
    travelled_since_ref_ = 0.0f;
    last_odom_x_ = last_odom_y_ = 0.0f;
    last_vision_seq_ = sensor_.vision_seq;
    tts_limbar_ = tts_coke_ = tts_football_ = tts_ball_ = tts_obstacle_ = false;

    // (2026-08-17 切换摔修复: set_pitch/stop走gamepad通道真机不可靠,
    //  改303静止命令; Stage3收尾已抬平停稳, 这里不再碰gamepad)
    motion_.select_gait(ServoGait::SLOW);
    motion_.set_step_height(kStandStepH, kStandStepH);
    motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);   // 303 静止抬平
    pitch_unlocked_ = false;
}

void Stage4Real::update_odometry() {
    // (2026-08-18 识别停点静止时不计里程: 静止踏步odom会漂移(实测8秒涨0.15m),
    //  计入travelled导致去程3.64m≠回程3.52m)
    if (state_ == State::SCAN_STOP) return;
    // (2026-08-17 odom_valid无人写恒false导致里程死) 改判: 定位数据有值即累里程
    if (sensor_.odom_x == 0.0f && sensor_.odom_y == 0.0f) return;
    if (!odom_initialized_) {
        last_odom_x_ = sensor_.odom_x;
        last_odom_y_ = sensor_.odom_y;
        odom_initialized_ = true;
        return;
    }
    const float dx = sensor_.odom_x - last_odom_x_;
    const float dy = sensor_.odom_y - last_odom_y_;
    const float step = std::sqrt(dx * dx + dy * dy);
    last_odom_x_ = sensor_.odom_x;
    last_odom_y_ = sensor_.odom_y;
    if (step >= 0.001f && step <= 0.30f) travelled_since_ref_ += step;
}

void Stage4Real::update_perception() {
    if (sensor_.vision_seq == 0 || sensor_.vision_seq == last_vision_seq_) return;
    last_vision_seq_ = sensor_.vision_seq;
    last_vision_time_ = std::chrono::steady_clock::now();
    if (state_ == State::WAIT_FOR_SENSORS) {
        const bool ready = sensor_.rgb_valid
                        && sensor_.abs_yaw != 0.0f   // (2026-08-17 必须等航向就绪! 否则entry_yaw=0→狗猛转圈摔倒)
                        && sensor_.odom_x != 0.0f
                        && motion_.gait_ready();
        ready_frames_ = ready ? ready_frames_ + 1 : 0;
    }
}
void Stage4Real::update_perception_in_state() { update_perception(); }

// ═══════════════════════════════════════════════════════════════
// 运动原语（yaw 源统一 abs_yaw，与 Stage1/2 同约定）
// ═══════════════════════════════════════════════════════════════
bool Stage4Real::walk_distance(float distance, float target_yaw, float speed) {
    speed = clamp01(speed, -0.55f, 0.55f);
    float yaw_err = norm_yaw(target_yaw - sensor_.abs_yaw);
    if (std::abs(yaw_err) < kYawDeadband) yaw_err = 0.0f;   // (2026-08-17 死区, Stage2同款)
    float yaw_cmd = clamp01(kYawKp * yaw_err, -kTurnRate, kTurnRate);
    if (travelled_since_ref_ >= distance) { motion_.stop(); return true; }
    // (2026-08-17 修复摔倒: set_velocity走gamepad通道会失控, 改303)
    // (2026-08-17 前进偏左: vy打底kFwdLatComp向右补; 4test版=0, 见hpp宏)
    motion_.set_walk_velocity_pitch(speed, kFwdLatComp, yaw_cmd, 0.0f);
    return false;
}

// 低姿行走 (2026-08-17 用户: 不用降身高改用“低头前进”; 201降高实测会掉地)
//   303 + rpy_des[1]=kLimbarPitch(0.25≈14°, 2026-08-17 用户: 大一些)
//   ⚠ 必须先破限 x_effect_scale_pos=+30 (PASS_LIMBAR进入时设置),
//     否则步态硬限±5.7°会把命令夹掉 (test17: 大命令无效)
bool Stage4Real::walk_low(float distance, float target_yaw, float speed) {
    speed = clamp01(speed, -0.55f, 0.55f);
    float yaw_err = norm_yaw(target_yaw - sensor_.abs_yaw);
    if (std::abs(yaw_err) < kYawDeadband) yaw_err = 0.0f;   // (2026-08-17 死区, Stage2同款)
    float yaw_cmd = clamp01(kYawKp * yaw_err, -kTurnRate, kTurnRate);
    if (travelled_since_ref_ - sub_start_travel_ >= distance) { motion_.stop(); return true; }
    // (2026-08-18 破限段: vel_cmd_scale被x_effect_scale_pos放大→横移补偿用更小kFwdLatCompLow防偏左过度; 4test版=0)
    motion_.set_walk_velocity_pitch(speed, kFwdLatCompLow, yaw_cmd, kLimbarPitch);
    return false;
}

bool Stage4Real::turn_to_yaw(float target_yaw) {
    target_yaw = norm_yaw(target_yaw + kTurnExtraRad);   // (2026-08-17 物理欠转2°补偿, Stage2同款)
    float yaw_err = norm_yaw(target_yaw - sensor_.abs_yaw);
    if (std::abs(yaw_err) <= kYawTol) { motion_.stop(); return true; }
    float cmd = clamp01(std::abs(yaw_err) * kYawKp, 0.05f, kTurnRate);   // (2026-08-22 下限0.10→0.05: 接近目标减速收尾, 防惯性过冲"转多")
    motion_.set_walk_velocity_pitch(0.0f, 0.0f, yaw_err > 0.0f ? cmd : -cmd, 0.0f);
    return false;
}

bool Stage4Real::align_to_target(float cx, float tol) {
    if (std::abs(cx) <= tol) { motion_.stop(); return true; }
    float cmd = clamp01(std::abs(cx) * 0.6f, 0.08f, kTurnRate);
    // (2026-08-18 修复转圈根因: 方向反了! cx>0偏右应右转(-cmd), cx<0偏左应左转(+cmd)。
    //  原逻辑 cx>0→左转→目标更右→永远转不到位→KNOCK对齐转圈(实测absYaw转229°))
    motion_.set_walk_velocity_pitch(0.0f, 0.0f, cx > 0.0f ? -cmd : cmd, 0.0f);
    return false;
}

// ═══════════════════════════════════════════════════════════════
// 恢复
// ═══════════════════════════════════════════════════════════════
void Stage4Real::enter_recovery() {
    if (state_ == State::RECOVERING || state_ == State::DONE) return;
    // (2026-08-17) 跌倒恢复前复原pitch限位, 防带着放大限位做恢复动作
    lane_pitch_restore();
    recovery_return_ = state_;
    state_ = State::RECOVERING;
    recovery_cmd_sent_ = false;
    gait_reengaged_ = false;
    recovery_start_ = std::chrono::steady_clock::now();
}

void Stage4Real::handle_recovering() {
    if (!recovery_cmd_sent_) {
        motion_.stop(); motion_.recovery();
        recovery_cmd_sent_ = true;
        recovery_start_ = std::chrono::steady_clock::now();
        return;
    }
    const auto elapsed = std::chrono::steady_clock::now() - recovery_start_;
    if (!gait_reengaged_) {
        const bool stable = std::abs(sensor_.roll) < kRecoverRollOk
                         && std::abs(sensor_.pitch) < kRecoverPitchOk;
        if (stable) {
            motion_.select_gait(ServoGait::SLOW);
            motion_.set_step_height(kStandStepH, kStandStepH);
            gait_reengaged_ = true;
            recovery_start_ = std::chrono::steady_clock::now();
        } else if (elapsed > kRecoveryStabilize) {
            if (++recovery_attempts_ >= kMaxRecoverAttempts) finish();
            else recovery_cmd_sent_ = false;
        }
        return;
    }
    if (motion_.gait_ready()) {
        recovery_cmd_sent_ = false; gait_reengaged_ = false; recovery_attempts_ = 0;
        state_ = recovery_return_;
        // 清里程重来当前段（防恢复期间里程跳变）
        odom_initialized_ = false;
        ref_x_ = last_odom_x_ = sensor_.odom_x;
        ref_y_ = last_odom_y_ = sensor_.odom_y;
        travelled_since_ref_ = 0.0f;
        sub_start_travel_ = 0.0f;
        sub_state_ = 0; state_frames_ = 0;
        last_vision_time_ = std::chrono::steady_clock::now();
        return;
    }
    if (elapsed > kGaitReengage) {
        if (++recovery_attempts_ >= kMaxRecoverAttempts) finish();
        else { gait_reengaged_ = false; recovery_cmd_sent_ = false; }
    }
}

void Stage4Real::finish() {
    motion_.select_gait(ServoGait::SLOW);
    motion_.set_step_height(kStandStepH, kStandStepH);
    motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);   // (2026-08-17 去gamepad, 改303静止)
    state_ = State::DONE; done_ = true;
}

bool Stage4Real::is_done() { return done_; }

// ═══════════════════════════════════════════════════════════════
// 2.8m段全程低头 (2026-08-18 用户): 进段破限 x_effect_scale_pos=+30,
//   walk_low 带 kLimbarPitch 低头走全程; 出段恢复默认限位
// ═══════════════════════════════════════════════════════════════
void Stage4Real::lane_pitch_unlock() {
    if (!pitch_unlocked_) {
        pitch_unlocked_ = true;
        motion_.set_user_param_double_lcm("x_effect_scale_pos", kPitchScaleUnlock);
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 进2.8m段: pitch限位已破限 x_effect_scale_pos=+%.0f\n", kPitchScaleUnlock);
        fflush(stderr);
#endif
    }
}
void Stage4Real::lane_pitch_restore() {
    if (pitch_unlocked_) {
        pitch_unlocked_ = false;
        motion_.set_user_param_double_lcm("x_effect_scale_pos", kPitchScaleDefault);
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 出2.8m段: pitch限位恢复默认 %.2f\n", kPitchScaleDefault);
        fflush(stderr);
#endif
    }
}

// ═══════════════════════════════════════════════════════════════
// 去程 2.8m：全程低头 + 可乐/足球撞击
// ═══════════════════════════════════════════════════════════════
void Stage4Real::lane_out() {
    // (2026-08-21 用户: 识别停点结束后固定前进0.5m, 不再是走完剩余段)
    const float lane_goal = post_scan_ ? kPostScanFwd : kLaneDist;
    if (travelled_since_ref_ >= lane_goal) {
        post_scan_ = false;
        lane_pitch_restore();   // (2026-08-18 离开2.8m段恢复限位)
        motion_.stop();
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 去程走满%.2fm → 掉头\n", travelled_since_ref_); fflush(stderr);
#endif
        state_ = State::TURN_BACK;
        sub_state_ = 0; state_frames_ = 0;
        return;
    }

    // ── 识别停点 (2026-08-17 用户: 剩0.3m处停5秒识别足球/可乐/橙球播报) ──
    if (!scan_done_out_ && travelled_since_ref_ >= kLaneDist - kScanStopMargin) {
        scan_done_out_ = true;
        scan_return_ = State::LANE_OUT;
        scan_frames_ = 0;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 去程剩%.2fm → 识别停点5s\n", kLaneDist - travelled_since_ref_);
        fflush(stderr);
#endif
        state_ = State::SCAN_STOP;
        state_frames_ = 0;
        return;
    }

    const auto age = std::chrono::steady_clock::now() - last_vision_time_;
    if (age <= kVisionTimeout) {
        // 限高杆 <0.8m → 只播报 (2026-08-18 用户: 全程已低头, 不再切低姿状态)
        if (sensor_.limbar_found && sensor_.limbar_dist > 0.0f
            && sensor_.limbar_dist < kLimbarTriggerDist) {
            if (!tts_limbar_) { motion_.speak("识别到限高杆"); tts_limbar_ = true; }
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S4] 去程限高杆 dist=%.2fm x=%.2f (全程低头, 直接过)\n",
                    sensor_.limbar_dist, sensor_.limbar_x); fflush(stderr);
#endif
        }
        // (2026-08-18 用户: 可乐/足球/橙球只在通道末端停点识别, 走动中不识别不撞击不播报)
        // 蓝色方块 (2026-08-18 用户: 伙伴写了识别, 我们补播报; 绕行待定)
        if (sensor_.obstacle_found) {
            if (!tts_obstacle_) { motion_.speak("识别到不可跨越障碍物"); tts_obstacle_ = true; }
        }
    } else {
        // 视觉超时：减速慢走（不平白停死）
        walk_low(kLaneDist, lane_yaw_, kLowSpeed);
        return;
    }
    walk_low(kLaneDist, lane_yaw_, kWalkSpeed);
}

// ═══════════════════════════════════════════════════════════════
// 限高杆：低姿（energy_saving → 身高0.13）前进1m → 起立
// ═══════════════════════════════════════════════════════════════
void Stage4Real::pass_limbar() {
    // (2026-08-18 保留函数: 全程低头方案下不再进入此状态)
    lane_pitch_unlock();
#ifdef DEBUG_STAGE
    if (state_frames_ % 100 == 0) {
        fprintf(stderr, "[S4] 低姿中 %.2f/%.2fm body_h=%.3f\n",
                travelled_since_ref_ - sub_start_travel_, kLimbarLowDist,
                sensor_.body_height); fflush(stderr);
    }
#endif
    if (walk_low(kLimbarLowDist, target_yaw_, kLowSpeed)) {
        motion_.stop();
        // 起立：energy=false 把 use_energy_saving_mode 置回1（身高低通回升）
        state_ = State::RISE;
        state_frames_ = 0;
    }
}

void Stage4Real::rise() {
    // 抬回头 (2026-08-17 低头方案收尾)
    if (state_frames_ < kRiseFrames) {
        ++state_frames_;
        motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }
    // (2026-08-18) 过杆完成 → 恢复pitch限位默认值
    lane_pitch_restore();
    // 回原来的去向（去程→LANE_OUT / 回程→LANE_BACK）
#ifdef DEBUG_STAGE
    fprintf(stderr, "[S4] 起立完成 → %s\n", state_name(rise_return_)); fflush(stderr);
#endif
    state_ = rise_return_;
    state_frames_ = 0;
}

// ═══════════════════════════════════════════════════════════════
// 撞击可乐/足球：对齐 → 冲 → 回 LANE_OUT
// ═══════════════════════════════════════════════════════════════
void Stage4Real::knock() {
    if (sub_state_ == 0) {
        // (2026-08-18 修复转圈: knock_cx_是进入时快照, align_to_target用它永不满足|cx|≤tol→原地转圈
        //  (实测absYaw 1.71→-1.24一直转); 改用实时球位置(转向中球回画面中央), 丢失/超时直接撞)
        const float cx = sensor_.coke_found ? sensor_.coke_x
                       : sensor_.football_found ? sensor_.football_x : 0.0f;
        if (align_to_target(cx) || state_frames_ > 120) {   // 1.2s超时兜底(10ms)
            sub_start_travel_ = travelled_since_ref_;
            sub_state_ = 1;
            state_frames_ = 0;
        }
        return;
    }
    // 冲撞：直到贴脸或冲满 kKnockDist
    const float dist = sensor_.coke_found ? sensor_.coke_dist
                      : sensor_.football_found ? sensor_.football_dist
                      : 1e9f;
#ifdef DEBUG_STAGE
    if (state_frames_ % 50 == 0) {
        fprintf(stderr, "[S4] 撞击中 冲%.2fm 目标dist=%.2fm\n",
                travelled_since_ref_ - sub_start_travel_, dist); fflush(stderr);
    }
#endif
    if (travelled_since_ref_ - sub_start_travel_ >= kKnockDist
        || (dist > 0.0f && dist <= kKnockMinDist)) {
        motion_.stop();
        state_ = State::LANE_OUT;
        sub_state_ = 0; state_frames_ = 0;
        return;
    }
    float yaw_err = norm_yaw(lane_yaw_ - sensor_.abs_yaw);
    float yaw_cmd = clamp01(kYawKp * yaw_err, -kTurnRate, kTurnRate);
    motion_.set_walk_velocity_pitch(kKnockSpeed, 0.0f, yaw_cmd, 0.0f);
}

// ═══════════════════════════════════════════════════════════════
// 回程 2.8m：只处理限高杆
// ═══════════════════════════════════════════════════════════════
void Stage4Real::lane_back() {
    const float back_dist = kLaneDistBack[round_count_];   // (2026-08-18 回程按轮次: 轮1=3.0 轮2=2.6 轮3=3.5)
    if (travelled_since_ref_ >= back_dist) {
        lane_pitch_restore();   // (2026-08-18 离开2.8m段恢复限位)
        motion_.stop();
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 回程走满%.2fm → %s\n", travelled_since_ref_,
                (round_count_ + 1 >= kMaxRounds) ? "左转离场" : "右转出通道"); fflush(stderr);
#endif
        // (2026-08-20 用户: 第3轮回程后左转前进3m离场, 不再右转出通道)
        state_ = (round_count_ + 1 >= kMaxRounds) ? State::TURN_L_FINAL : State::TURN_R_OUT;
        sub_state_ = 0; state_frames_ = 0;
        return;
    }

    const auto age = std::chrono::steady_clock::now() - last_vision_time_;
    if (age <= kVisionTimeout
        && sensor_.limbar_found && sensor_.limbar_dist > 0.0f
        && sensor_.limbar_dist < kLimbarTriggerDist) {
        // (2026-08-18 全程已低头, 只播报不切状态)
        if (!tts_limbar_) { motion_.speak("识别到限高杆"); tts_limbar_ = true; }
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 回程限高杆 dist=%.2fm x=%.2f (全程低头, 直接过)\n",
                sensor_.limbar_dist, sensor_.limbar_x); fflush(stderr);
#endif
    }
    // (2026-08-18 用户: 橙球只在停点识别, 回程走动中不播报)
    // 回程蓝色方块播报 (2026-08-18)
    if (age <= kVisionTimeout && sensor_.obstacle_found) {
        if (!tts_obstacle_) { motion_.speak("识别到不可跨越障碍物"); tts_obstacle_ = true; }
    }
    if (age > kVisionTimeout) {
        walk_low(back_dist, back_yaw_, kLowSpeed);
        return;
    }
    walk_low(back_dist, back_yaw_, kWalkSpeed);
}

// ═══════════════════════════════════════════════════════════════// 识别停点 (2026-08-17 用户: 静止5秒播报可乐/足球/橙球, 再走完剩余0.3m)
// ═══════════════════════════════════════════════════════════
void Stage4Real::scan_stop() {
    motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);   // 静止
    if (!tts_coke_ && sensor_.coke_found) {
        motion_.speak("识别到可乐瓶"); tts_coke_ = true;
    }
    if (!tts_football_ && sensor_.football_found) {
        motion_.speak("识别到足球"); tts_football_ = true;
    }
    if (!tts_ball_ && sensor_.ball_found && sensor_.ball_dist > 0.0f
        && sensor_.ball_dist < 1.0f) {
        motion_.speak("识别到橙色球"); tts_ball_ = true;
    }
    // (2026-08-18 用户: 停点只识别可乐/足球/橙色球, 不识别蓝色块)
    if (++scan_frames_ >= kScanHoldFrames) {
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 识别停点完成 → %s\n", state_name(scan_return_));
        fflush(stderr);
#endif
        // (2026-08-21 用户: 识别完不管结果, 固定前进0.5m再掉头)
        // 重置里程基准(静止odom漂移不计入), 回 LANE_OUT 走 0.5m
        post_scan_ = true;
        ref_x_ = last_odom_x_ = sensor_.odom_x;
        ref_y_ = last_odom_y_ = sensor_.odom_y;
        travelled_since_ref_ = 0.0f;
        state_ = State::LANE_OUT;
        scan_frames_ = 0;
        state_frames_ = 0;
    }
}

// ═══════════════════════════════════════════════════════════// 主循环
// ═══════════════════════════════════════════════════════════════
void Stage4Real::run() {
    if (done_) return;
    // ── 心跳诊断 (2026-08-17 排查趴下: 每20帧打印, 看control_loop是否活着) ──
    {
        static int hb_ = 0;
        if (++hb_ % 20 == 0) {
            fprintf(stderr, "[S4] hb state=%s travelled=%.3f odom=(%.2f,%.2f) absYaw=%.2f gait_ready=%d\n",
                    state_name(state_), travelled_since_ref_,
                    sensor_.odom_x, sensor_.odom_y, sensor_.abs_yaw,
                    motion_.gait_ready() ? 1 : 0);
            fflush(stderr);
        }
    }
    update_odometry();

    if (state_ != State::RECOVERING && state_ != State::DONE
        && state_ != State::WAIT_FOR_SENSORS) {
        if (motion_.servo_fault()) { enter_recovery(); return; }
        if (std::abs(sensor_.roll) > kFallRollThresh
         || std::abs(sensor_.pitch) > kFallPitchThresh) { enter_recovery(); return; }
    }
    if (state_ == State::RECOVERING) { handle_recovering(); return; }

    if (state_ == State::WAIT_FOR_SENSORS) {
        // (2026-08-17 切换摔修复: stop走gamepad通道, 改303静止保持伺服模式)
        motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);
        update_perception();
        const bool ready = sensor_.rgb_valid
                        && sensor_.abs_yaw != 0.0f   // (2026-08-17 必须等航向就绪)
                        && sensor_.odom_x != 0.0f
                        && motion_.gait_ready();
        if (!ready) ready_frames_ = 0;
        if (ready && ready_frames_ >= kReadyFrames) {
            entry_yaw_ = sensor_.abs_yaw;
            lane_yaw_  = norm_yaw(entry_yaw_ - 1.5708f);   // 右转90°进通道
            back_yaw_  = norm_yaw(entry_yaw_ + 1.5708f);   // 掉头180°返回朝向
            odom_initialized_ = false;
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            round_count_ = 0;
            tts_limbar_ = tts_coke_ = tts_football_ = tts_ball_ = tts_obstacle_ = false;
            scan_done_out_ = false;
            state_frames_ = 0; sub_state_ = 0;
            state_ = State::FWD_1M;
        }
        return;
    }

    update_perception_in_state();
    ++state_frames_;

#ifdef DEBUG_STAGE
    // 状态切换日志 (2026-08-17)
    if (state_ != last_log_state_) {
        last_log_state_ = state_;
        fprintf(stderr, "[S4] 状态→ %s (第%d轮 已走%.2f/%.2fm)\n",
                state_name(state_), round_count_ + 1,
                travelled_since_ref_,
                (state_ == State::LANE_OUT || state_ == State::LANE_BACK
                 || state_ == State::PASS_LIMBAR || state_ == State::KNOCK) ? kLaneDist : kFwdDist[round_count_]);
        fflush(stderr);
    }
#endif

    switch (state_) {
        case State::FWD_1M: {
            // (2026-08-18 用户: 三轮起步分别走1m/0.8m/1m)
            if (walk_distance(kFwdDist[round_count_], entry_yaw_, kWalkSpeed)) {
                motion_.stop();
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::TURN_R_IN;
            }
            return;
        }
        case State::TURN_R_IN: {
            if (turn_to_yaw(lane_yaw_)) {
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                sub_start_travel_ = 0.0f;
                lane_pitch_unlock();   // (2026-08-18 进2.8m段全程低头)
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::LANE_OUT;
            }
            return;
        }
        case State::LANE_OUT: {
            lane_out();
            return;
        }
        case State::PASS_LIMBAR: {
            pass_limbar();
            return;
        }
        case State::RISE: {
            rise();
            return;
        }
        case State::KNOCK: {
            knock();
            return;
        }
        case State::SCAN_STOP: {
            scan_stop();
            return;
        }
        case State::TURN_BACK: {
            // (2026-08-18 用户: 180°掉头多转点试→用 back_yaw_ - kTurnExtraRad 抵消-2°补偿, 转满180°)
            if (turn_to_yaw(back_yaw_ - kTurnExtraRad)) {
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                sub_start_travel_ = 0.0f;
                lane_pitch_unlock();   // (2026-08-18 回程2.8m同样全程低头)
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::LANE_BACK;
            }
            return;
        }
        case State::LANE_BACK: {
            lane_back();
            return;
        }
        case State::TURN_R_OUT: {
            // (2026-08-22 用户: 出通道改左转90°(原右转回entry), 每轮起步方向反转180°)
            if (turn_to_yaw(norm_yaw(entry_yaw_ + 3.14159265f))) {
                entry_yaw_ = norm_yaw(entry_yaw_ + 3.14159265f);
                lane_yaw_  = norm_yaw(entry_yaw_ - 1.5708f);   // 右转90°进通道
                back_yaw_  = norm_yaw(entry_yaw_ + 1.5708f);   // 掉头返回朝向
                ++round_count_;
                fprintf(stderr, "\033[1;35m[S4] 第%d轮完成\033[0m\n", round_count_);
                if (round_count_ >= kMaxRounds) { finish(); return; }
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                state_frames_ = 0; sub_state_ = 0;
                tts_limbar_ = tts_coke_ = tts_football_ = tts_ball_ = tts_obstacle_ = false;
                scan_done_out_ = false;   // (2026-08-18 回程停点已删, 只去程停)
                state_ = State::FWD_1M;
            }
            return;
        }
        case State::TURN_L_FINAL: {   // (2026-08-20 第3轮回程后: 左转90°离场)
            if (turn_to_yaw(norm_yaw(back_yaw_ + 1.5708f))) {
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::FWD_FINAL;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S4] 左转完成, 离场前进3m\n"); fflush(stderr);
#endif
            }
            return;
        }
        case State::FWD_FINAL: {   // (2026-08-20 离场前进; 2026-08-21 3m→2.5m, 末尾加左转90°收尾)
            if (walk_distance(kFinalFwd, norm_yaw(back_yaw_ + 1.5708f), kWalkSpeed)) {
                motion_.stop();
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::TURN_END;
            }
            return;
        }
        case State::TURN_END: {   // (2026-08-21 离场末尾左转90° → DONE)
            if (turn_to_yaw(norm_yaw(entry_yaw_ + 1.5708f))) {
                finish();
            }
            return;
        }
        default: return;
    }
}
