#include "cyberdog_race/stages/real/stage4_real.hpp"

#include "cyberdog_race/debug_config.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace {
inline float clamp01(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}
}  // namespace

float Stage4Real::clamp(float v, float lo, float hi) { return clamp01(v, lo, hi); }

void Stage4Real::init() {
    state_ = State::WAIT_FOR_SENSORS;
    done_ = false;
    odom_initialized_ = false;
    crouch_active_ = false;
    recovery_cmd_sent_ = false;
    gait_reengaged_ = false;
    recovery_attempts_ = 0;
    ready_frames_ = 0;
    detect_wait_ = 0;
    lost_frames_ = 0;
    state_frames_ = 0;
    sub_state_ = 0;
    last_vision_seq_ = sensor_.vision_seq;
    entry_yaw_ = 0.0f;
    channel_yaw_ = 0.0f;
    switch_yaw_ = 0.0f;
    channel_count_ = 0;
    in_channel_ = false;
    entrance_x_ = entrance_y_ = 0.0f;
    limbars_on_path_ = 0;
    last_limbar_dist_ = 0.0f;
    dashed_found_ = false;
    dashed_yaw_ = 0.0f;
    obstacle_side_ = 0;
    ref_x_ = ref_y_ = 0.0f;
    travelled_since_ref_ = 0.0f;
    total_travelled_ = 0.0f;
    last_odom_x_ = last_odom_y_ = 0.0f;
    target_yaw_ = 0.0f;
    clearing_after_handle_ = false;
    processed_count_ = 0;

    // 语音播报去重标志初始化
    tts_limbar_ = tts_obstacle_ = tts_coke_ = tts_ball_ = tts_football_ = false;

    motion_.select_gait(ServoGait::SLOW);
    motion_.set_step_height(kStandStepH, kStandStepH);
    motion_.set_pitch(0.0f);
    motion_.stop();
}

void Stage4Real::update_odometry() {
    if (!sensor_.odom_valid) return;
    if (!odom_initialized_) {
        last_odom_x_ = sensor_.odom_x;
        last_odom_y_ = sensor_.odom_y;
        ref_x_ = sensor_.odom_x;
        ref_y_ = sensor_.odom_y;
        odom_initialized_ = true;
        return;
    }
    const float dx = sensor_.odom_x - last_odom_x_;
    const float dy = sensor_.odom_y - last_odom_y_;
    const float step = std::sqrt(dx*dx + dy*dy);
    last_odom_x_ = sensor_.odom_x;
    last_odom_y_ = sensor_.odom_y;
    if (step >= 0.001f && step <= 0.30f) {
        travelled_since_ref_ += step;
        total_travelled_     += step;
    }
}

void Stage4Real::update_perception() {
    if (sensor_.vision_seq == 0 || sensor_.vision_seq == last_vision_seq_) return;
    last_vision_seq_ = sensor_.vision_seq;
    last_vision_time_ = std::chrono::steady_clock::now();
    if (state_ == State::WAIT_FOR_SENSORS) {
        const bool ready = sensor_.rgb_valid && sensor_.imu_valid
                        && sensor_.odom_valid && motion_.gait_ready();
        ready_frames_ = ready ? ready_frames_ + 1 : 0;
    }
}
void Stage4Real::update_perception_in_state() { update_perception(); }

bool Stage4Real::walk_distance(float distance, float target_yaw, float speed) {
    speed = clamp01(speed, -0.55f, 0.55f);
    float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
    float yaw_cmd = clamp01(kYawKp * yaw_err, -kTurnRate, kTurnRate);
    if (travelled_since_ref_ >= distance) { motion_.stop(); return true; }
    motion_.set_velocity(speed, 0.0f, yaw_cmd);
    return false;
}

// 横移/斜向行走：x_dir/y_dir 单位向量（已归一化），保持 target_yaw 航向闭环
//   x_dir=1,y_dir=0   → 纯前进（walk_distance 同款）
//   x_dir=0,y_dir=1   → 纯右横移
//   x_dir=0,y_dir=-1  → 纯左横移
//   x_dir=0.707,y_dir=-0.707 → 左前 45° 斜向
bool Stage4Real::walk_xy_distance(float distance, float x_dir, float y_dir,
                                   float target_yaw, float speed) {
    speed = clamp01(speed, -0.55f, 0.55f);
    // 归一化方向向量（防传参未归一化）
    const float nrm = std::hypot(x_dir, y_dir);
    if (nrm < 1e-4f) { motion_.stop(); return true; }
    x_dir /= nrm; y_dir /= nrm;
    float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
    float yaw_cmd = clamp01(kYawKp * yaw_err, -kTurnRate, kTurnRate);
    if (travelled_since_ref_ >= distance) { motion_.stop(); return true; }
    motion_.set_velocity(speed * x_dir, speed * y_dir, yaw_cmd);
    return false;
}

bool Stage4Real::turn_to_yaw(float target_yaw) {
    float yaw_err = norm_yaw(target_yaw - sensor_.yaw);
    if (std::abs(yaw_err) <= kYawTol) { motion_.stop(); return true; }
    float cmd = clamp01(std::abs(yaw_err) * kYawKp, 0.10f, kTurnRate);
    motion_.set_velocity(0.0f, 0.0f, yaw_err > 0.0f ? cmd : -cmd);
    return false;
}

bool Stage4Real::align_to_target(float cx, float tol) {
    if (std::abs(cx) <= tol) { motion_.stop(); return true; }
    float cmd = clamp01(std::abs(cx) * 0.6f, 0.08f, kTurnRate);
    motion_.set_velocity(0.0f, 0.0f, cx > 0.0f ? cmd : -cmd);
    return false;
}

void Stage4Real::enter_recovery() {
    if (state_ == State::RECOVERING || state_ == State::DONE) return;
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
            motion_.set_step_height(crouch_active_ ? kCrouchStepH : kStandStepH,
                                    crouch_active_ ? kCrouchStepH : kStandStepH);
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
        state_ = State::NAVIGATE_CH;
        odom_initialized_ = false; clearing_after_handle_ = false;
        last_vision_time_ = std::chrono::steady_clock::now();
        return;
    }
    if (elapsed > kGaitReengage) {
        if (++recovery_attempts_ >= kMaxRecoverAttempts) finish();
        else { gait_reengaged_ = false; recovery_cmd_sent_ = false; }
    }
}

void Stage4Real::finish() {
    crouch_active_ = false; in_channel_ = false;
    motion_.select_gait(ServoGait::SLOW);
    motion_.set_step_height(kStandStepH, kStandStepH);
    motion_.set_pitch(0.0f); motion_.stop();
    state_ = State::DONE; done_ = true;
}

bool Stage4Real::is_done() { return done_; }

// ───────────────────────────────────────────────────────────────
//  finish_handle：任务(可乐/橙球/足球)完成后的收尾
//    need_return=true（任务目标）→ RETURN_ENTRANCE：转180°原路返回入口
//    need_return=false（限高杆等）→ 继续 NAVIGATE_CH
// ───────────────────────────────────────────────────────────────
void Stage4Real::finish_handle(bool need_return) {
    clearing_after_handle_ = false;
    ++processed_count_;
    sub_state_ = 0; state_frames_ = 0;
    detect_wait_ = 0; lost_frames_ = 0;

    if (need_return) {
        state_ = State::RETURN_ENTRANCE;
        ref_x_ = last_odom_x_ = sensor_.odom_x;
        ref_y_ = last_odom_y_ = sensor_.odom_y;
        travelled_since_ref_ = 0.0f;
    } else {
        state_ = State::NAVIGATE_CH;
        ref_x_ = last_odom_x_ = sensor_.odom_x;
        ref_y_ = last_odom_y_ = sensor_.odom_y;
        travelled_since_ref_ = 0.0f;
    }
}

// ═══════════════════════════════════════════════════════════════════
// 主循环
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::run() {
    if (done_) return;
    update_odometry();

    if (state_ != State::RECOVERING && state_ != State::DONE
        && state_ != State::WAIT_FOR_SENSORS) {
        if (motion_.servo_fault()) { enter_recovery(); return; }
        if (std::abs(sensor_.roll) > kFallRollThresh
         || std::abs(sensor_.pitch) > kFallPitchThresh) { enter_recovery(); return; }
    }
    if (state_ == State::RECOVERING) { handle_recovering(); return; }

    if (state_ == State::WAIT_FOR_SENSORS) {
        motion_.stop();
        update_perception();
        const bool ready = sensor_.rgb_valid && sensor_.imu_valid
                        && sensor_.odom_valid && motion_.gait_ready();
        if (!ready) ready_frames_ = 0;
        if (ready && ready_frames_ >= kReadyFrames) {
            entry_yaw_ = sensor_.yaw;
            channel_yaw_ = entry_yaw_;                    // 通道内上行
            switch_yaw_ = norm_yaw(entry_yaw_ + 1.5708f); // 切换时向右走
            odom_initialized_ = false;
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            total_travelled_ = 0.0f;
            channel_count_ = 0;
            clearing_after_handle_ = false;
            state_ = State::INIT_ALIGN;
        }
        return;
    }

    update_perception_in_state();
    ++state_frames_;
    const auto age = std::chrono::steady_clock::now() - last_vision_time_;

    switch (state_) {
        case State::INIT_ALIGN:         init_align();         return;
        case State::NAVIGATE_CH:
        {
            if (clearing_after_handle_) {
                if (walk_distance(kClearAfterHandle, channel_yaw_, kWalkSpeed)) {
                    clearing_after_handle_ = false;
                    detect_wait_ = 0; lost_frames_ = 0;
                }
                return;
            }
            // 通道尽头黄实线边界 → 返回入口（视为通道走完）
            if (sensor_.divider_found && !sensor_.divider_is_dashed
                && sensor_.divider_dist > 0 && sensor_.divider_dist < kDividerStopDist) {
                state_ = State::RETURN_ENTRANCE;
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                return;
            }
            if (age > kVisionTimeout) { motion_.stop(); return; }

            // 优先级：限高杆 → 蓝障碍 → 目标
            if (sensor_.limbar_found && sensor_.limbar_dist < 1.5f) {
                if (!tts_limbar_) { motion_.speak("识别到限高杆"); tts_limbar_ = true; }
                target_yaw_ = sensor_.yaw; sub_state_ = 0; state_frames_ = 0;
                state_ = State::PASS_LIMBAR;
                return;
            }
            if (sensor_.obstacle_found && sensor_.obstacle_dist < 1.5f) {
                if (!tts_obstacle_) { motion_.speak("识别到无法跨越障碍"); tts_obstacle_ = true; }
                dashed_found_ = false; obstacle_side_ = 0;
                sub_state_ = 0; state_frames_ = 0;
                state_ = State::OBSTACLE_SCAN;
                return;
            }
            if (sensor_.coke_found && sensor_.coke_dist < 1.2f) {
                if (!tts_coke_) { motion_.speak("识别到可乐瓶"); tts_coke_ = true; }
                target_yaw_ = channel_yaw_; sub_state_ = 0; state_frames_ = 0;
                state_ = State::HANDLE_COKE;
                return;
            }
            if (sensor_.ball_found && sensor_.ball_dist < 1.0f) {
                if (!tts_ball_) { motion_.speak("识别到橙色小球"); tts_ball_ = true; }
                target_yaw_ = channel_yaw_; sub_state_ = 0; state_frames_ = 0;
                state_ = State::HANDLE_ORANGE_BALL;
                return;
            }
            if (sensor_.football_found && sensor_.football_dist < 1.2f) {
                if (!tts_football_) { motion_.speak("识别到足球"); tts_football_ = true; }
                target_yaw_ = channel_yaw_; sub_state_ = 0; state_frames_ = 0;
                state_ = State::HANDLE_FOOTBALL;
                return;
            }

            motion_.set_velocity(kNavigateSpeed, 0.0f, 0.0f);
            if (++detect_wait_ > kDetectMaxFrames * 4) {
                state_ = State::RETURN_ENTRANCE;
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
            }
            return;
        }
        case State::SWITCH_CHANNEL:     switch_channel();     return;
        case State::PASS_LIMBAR:        pass_limbar();        return;
        case State::HANDLE_COKE:        handle_coke();        return;
        case State::HANDLE_ORANGE_BALL: handle_orange_ball(); return;
        case State::HANDLE_FOOTBALL:    handle_football();    return;
        case State::OBSTACLE_SCAN:      obstacle_scan();      return;
        case State::OBSTACLE_CROSS:     obstacle_cross();     return;
        case State::RETURN_ENTRANCE:    return_entrance();    return;
        case State::SEEK_BRIDGE:        seek_bridge();        return;
        default: return;
    }
}

// ═══════════════════════════════════════════════════════════════════
// INIT_ALIGN：3-4衔接
//   正式比赛：phase 0 左转90°→phase 1 走2.7m→phase 2 右转90°→phase 3 蹲→phase 4 走0.8m进通道→起
//   DEBUG_SINGLE_STAGE==4 单测模式：狗已正对 Stage4 起点（init 中 channel_yaw_=sensor_.yaw）
//                    跳过左转+2.7m+右转 → 直接蹲→走0.8m进通道 → NAVIGATE_CH
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::init_align() {
#if defined(DEBUG_SINGLE_STAGE) && DEBUG_SINGLE_STAGE == 4
    if (sub_state_ == 0) {
        RCLCPP_WARN(rclcpp::get_logger("stage4_real"),
                    "[S4] SINGLE_STAGE 模式：跳过 左转+2.7m+右转，直接进入通道1");
        state_frames_ = 0; sub_state_ = 3;  // 直接去"蹲下→走入通道"
        return;
    }
#endif
    if (sub_state_ == 0) {
        const float left_yaw = norm_yaw(entry_yaw_ - 1.5708f);
        if (turn_to_yaw(left_yaw)) {
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 1;
        }
        return;
    }
    if (sub_state_ == 1) {
        const float left_yaw = norm_yaw(entry_yaw_ - 1.5708f);
        if (walk_distance(kInitWalkDist, left_yaw, kWalkSpeed)) {
            motion_.stop();
            state_frames_ = 0; sub_state_ = 2;
        }
        return;
    }
    if (sub_state_ == 2) {
        if (turn_to_yaw(channel_yaw_)) {
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 3;
        }
        return;
    }
    if (sub_state_ == 3) {
        // 蹲下到35cm + 走入通道80cm
        crouch_active_ = true;
        motion_.set_step_height(kCrouchStepH, kCrouchStepH);
        motion_.stop();
        if (++state_frames_ > kCrouchFrames
            && sensor_.body_height < kCrouchHeight + 0.03f) {
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 4;
        }
        return;
    }
    if (sub_state_ == 4) {
        // 走入通道，记录入口位置（返回用）
        if (walk_distance(kChannelEnterDist, channel_yaw_, kCrawlSpeed)) {
            in_channel_ = true;
            crouch_active_ = false;
            motion_.set_step_height(kStandStepH, kStandStepH);
            entrance_x_ = last_odom_x_ = sensor_.odom_x;
            entrance_y_ = last_odom_y_ = sensor_.odom_y;
            ref_x_ = entrance_x_; ref_y_ = entrance_y_;
            travelled_since_ref_ = 0.0f;
            total_travelled_ = 0.0f;
            limbars_on_path_ = 0;
            state_frames_ = 0; sub_state_ = 0;
            state_ = State::NAVIGATE_CH;
        }
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════
// SWITCH_CHANNEL：通道切换（入口→左转90°→走1m→左转90°→进下通道）
//   phase 0: 左转90°（面向主通道向右方向 switch_yaw_）
//   phase 1: 走1m
//   phase 2: 左转90°（面向通道上行 channel_yaw_）
//   phase 3: 蹲下 + 走入通道80cm
//   phase 4: 起立 记入口 → NAVIGATE_CH
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::switch_channel() {
    if (sub_state_ == 0) {
        if (turn_to_yaw(switch_yaw_)) {
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 1;
        }
        return;
    }
    if (sub_state_ == 1) {
        if (walk_distance(kSwitchWalkDist, switch_yaw_, kWalkSpeed)) {
            motion_.stop();
            state_frames_ = 0; sub_state_ = 2;
        }
        return;
    }
    if (sub_state_ == 2) {
        if (turn_to_yaw(channel_yaw_)) {
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 3;
        }
        return;
    }
    if (sub_state_ == 3) {
        crouch_active_ = true;
        motion_.set_step_height(kCrouchStepH, kCrouchStepH);
        motion_.stop();
        if (++state_frames_ > kCrouchFrames
            && sensor_.body_height < kCrouchHeight + 0.03f) {
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 4;
        }
        return;
    }
    if (sub_state_ == 4) {
        if (walk_distance(kChannelEnterDist, channel_yaw_, kCrawlSpeed)) {
            in_channel_ = true;
            crouch_active_ = false;
            motion_.set_step_height(kStandStepH, kStandStepH);
            ++channel_count_;
            entrance_x_ = last_odom_x_ = sensor_.odom_x;
            entrance_y_ = last_odom_y_ = sensor_.odom_y;
            ref_x_ = entrance_x_; ref_y_ = entrance_y_;
            travelled_since_ref_ = 0.0f;
            total_travelled_ = 0.0f;
            limbars_on_path_ = 0;
            // 新通道：复位语音播报去重（同一目标在新通道要重新播报）
            tts_limbar_ = tts_obstacle_ = tts_coke_ = tts_ball_ = tts_football_ = false;
            state_frames_ = 0; sub_state_ = 0;
            state_ = State::NAVIGATE_CH;
        }
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════
// PASS_LIMBAR：限高杆
//   phase 0: 对齐限高杆 → 若视野同时看到足球，target_yaw_ 朝足球做预对齐 → 蹲35cm
//   phase 1: 等蹲到位 → 清零里程
//   phase 2: 走50cm矮身通过（全程：若看到足球→按足球cx动态转向对齐；否则→target_yaw_闭环）
//            这样从限高杆下出来时狗身就正对足球
//   phase 3: 起立 → finish_handle(false) 继续 NAVIGATE_CH
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::pass_limbar() {
    if (sub_state_ == 0) {
        // Step 1: 先把身体对准限高杆   
        if (!align_to_target(sensor_.limbar_x)) return;
        // Step 2: 对齐完如果视野里已经看得到足球（cx 归一化，阈值放宽到 0.5 以内） 
        //         → 以足球 cx 做一次精对齐，保证正对杆后目标
        if (sensor_.football_found && std::abs(sensor_.football_x) < 0.5f) {
            const float fcx = sensor_.football_x;
            if (std::abs(fcx) <= 0.12f) {
                // 足球已在中心，记录当前朝向作为行走基准
                target_yaw_ = sensor_.yaw;
                crouch_active_ = true;
                motion_.set_step_height(kCrouchStepH, kCrouchStepH);
                state_frames_ = 0; sub_state_ = 1;
            } else {
                // 还没对准足球，逐帧微调转向（步幅小，限高杆还在附近不易蹭）
                const float cmd = clamp01(std::abs(fcx) * 0.5f, 0.06f, kTurnRate * 0.6f);
                motion_.set_velocity(0.0f, 0.0f, fcx > 0.0f ? cmd : -cmd);
            }
            return;
        }
        // 没看到足球：保留限高杆对齐朝向，直接进入蹲下
        target_yaw_ = sensor_.yaw;
        crouch_active_ = true;
        motion_.set_step_height(kCrouchStepH, kCrouchStepH);
        state_frames_ = 0; sub_state_ = 1;
        return;
    }
    if (sub_state_ == 1) {
        motion_.stop();
        if (++state_frames_ > kCrouchFrames
            && sensor_.body_height < kCrouchHeight + 0.03f) {
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 2;
        }
        return;
    }
    if (sub_state_ == 2) {
        if (sensor_.tof_available && sensor_.tof_clearance < kBridgeClearance) {
            motion_.stop(); return;
        }
        if (travelled_since_ref_ >= kLimbarWalkDist) {
            // 走完0.5m，起立
            crouch_active_ = false;
            motion_.set_step_height(kStandStepH, kStandStepH);
            ++limbars_on_path_;
            state_frames_ = 0; sub_state_ = 3;
            motion_.stop();
            return;
        }

        // ── 矮身行走阶段：yaw 命令分两种（优先对齐足球cx） ──
        float yaw_cmd = 0.0f;
        const bool see_football = sensor_.football_found
                                  && sensor_.football_dist > 0
                                  && sensor_.football_dist < 3.0f;  // 球在限高杆后，距离可能较远
        if (see_football) {
            // 有足球 → 按 cx 动态转向对齐（比普通align增益稍低，蹲着走更稳）
            const float fcx = sensor_.football_x;
            const float gain = 0.5f;  // 转向增益（蹲着别转太猛）
            yaw_cmd = clamp01(fcx * gain, -kTurnRate * 0.7f, kTurnRate * 0.7f);
        } else {
            // 没足球 → 保持进入时的 target_yaw_ 航向闭环（限高杆本身的朝向）
            const float yaw_err = norm_yaw(target_yaw_ - sensor_.yaw);
            yaw_cmd = clamp01(kYawKp * yaw_err, -kTurnRate, kTurnRate);
        }
        motion_.set_velocity(kCrawlSpeed, 0.0f, yaw_cmd);
        return;
    }
    if (sub_state_ == 3) {
        motion_.stop();
        if (++state_frames_ > kCrouchFrames
            && sensor_.body_height > kStandHeight - 0.03f) {
            // 起身后如果之前在杆下已经对准过足球，就不再需要二次对齐
            finish_handle(false);
        }
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════
// HANDLE_COKE：对齐→撞击1.2m→返回入口
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::handle_coke() {
    if (sub_state_ == 0) {
        if (align_to_target(sensor_.coke_x)) {
            target_yaw_ = sensor_.yaw;
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            sub_state_ = 1;
        }
        return;
    }
    if (sub_state_ == 1) {
        if (walk_distance(kCokeKnock, target_yaw_, kKnockSpeed)) {
            finish_handle(true);
        }
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════
// HANDLE_ORANGE_BALL：对齐→推0.3m→退0.2m→返回入口
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::handle_orange_ball() {
    if (sub_state_ == 0) {
        if (align_to_target(sensor_.ball_x)) {
            target_yaw_ = sensor_.yaw;
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            sub_state_ = 1;
        }
        return;
    }
    if (sub_state_ == 1) {
        if (walk_distance(kOrangePokeDist, target_yaw_, kWalkSpeed * 0.6f)) {
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            sub_state_ = 2;
        }
        return;
    }
    if (sub_state_ == 2) {
        if (walk_distance(kOrangeBackDist, target_yaw_, -kBackSpeed)) {
            finish_handle(true);
        }
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════
// HANDLE_FOOTBALL：对齐→撞球1.5m→返回入口
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::handle_football() {
    if (sub_state_ == 0) {
        if (align_to_target(sensor_.football_x)) {
            target_yaw_ = sensor_.yaw;
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            sub_state_ = 1;
        }
        return;
    }
    if (sub_state_ == 1) {
        if (walk_distance(kFootballKickDist, target_yaw_, kKnockSpeed)) {
            finish_handle(true);
        }
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════
// OBSTACLE_SCAN：蓝障碍 — 停稳 + 判断虚线方向（横移方案，不再左右扫线90°）
//   phase 0: 记录当前朝向 + 原地停稳 0.2s
//   phase 1: 判断虚线在哪一侧（divider_x 正=虚线在右→右侧跨越；负=在左→左侧跨越）
//            若没看到虚线：兜底按障碍物 obstacle_x 反向（障碍在右往左绕）
//   phase 2: → OBSTACLE_CROSS 横移绕行
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::obstacle_scan() {
    if (sub_state_ == 0) {
        // 原地踏步停稳，记录当前朝向（身体不要转向，保持正对通道）
        target_yaw_ = sensor_.yaw;
        motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, kStandStepH);
        if (++state_frames_ > 20) {
            state_frames_ = 0; sub_state_ = 1;
        }
        return;
    }
    if (sub_state_ == 1) {
        // 每个 tick 持续检查虚线并判断方向
        // 虚线（divider_is_dashed=true）距离合理时：按 divider_x 左右决定横移方向
        //   divider_x>0 → 虚线在画面右侧 → 狗向右平移 (+y)
        //   divider_x<0 → 虚线在画面左侧 → 狗向左平移 (-y)
        // 兜底：完全没虚线时，从障碍物反方向走（避免撞实黄线）
        if (sensor_.divider_found && sensor_.divider_is_dashed
            && sensor_.divider_dist > 0 && sensor_.divider_dist < 2.0f) {
            dashed_found_ = true;
            obstacle_side_ = (sensor_.divider_x >= 0.0f) ? +1 : -1;
        } else {
            dashed_found_ = false;
            // 没虚线兜底：障碍物在右(cx>0)→左绕；障碍物在左→右绕
            obstacle_side_ = (sensor_.obstacle_x >= 0.0f) ? -1 : +1;
        }
        sub_state_ = 2;
        return;
    }
    if (sub_state_ == 2) {
        // 去 OBSTACLE_CROSS 执行横移绕行
        sub_state_ = 0; state_frames_ = 0;
        ref_x_ = last_odom_x_ = sensor_.odom_x;
        ref_y_ = last_odom_y_ = sensor_.odom_y;
        travelled_since_ref_ = 0.0f;
        state_ = State::OBSTACLE_CROSS;
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════
// OBSTACLE_CROSS：蓝障碍 — 横移绕行四步（身体始终正对通道朝向，不变朝向）
//   phase 0: 纯横移 kObsShift (50cm) 到虚线另一侧邻道
//             obstacle_side_=+1 → 右横移 (0, +1)
//             obstacle_side_=-1 → 左横移 (0, -1)
//   phase 1: 直走 kObsFwdStep (20cm) ，在邻道上越过障碍正前方
//   phase 2: 左前/右前 45° 斜向 kObsDiagDist (~55cm) 回到障碍物所在赛道
//             obstacle_side_=+1 → 左前 45° (x+, y-)
//             obstacle_side_=-1 → 右前 45° (x+, y+)
//   phase 3: 身体摆正（其实全程 yaw 已闭环，这里直接回 NAVIGATE_CH）
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::obstacle_cross() {
    // 全程保持 channel_yaw_ 朝向闭环，避免身体偏转
    const float face = channel_yaw_;

    if (sub_state_ == 0) {
        // Phase 1: 左右横移 50cm 跨过虚线
        //   +1 → 右横移 (x=0, y=+1)
        //   -1 → 左横移 (x=0, y=-1)
        const float xd = 0.0f;
        const float yd = static_cast<float>(obstacle_side_);
        if (walk_xy_distance(kObsShift, xd, yd, face, kObsShiftSpeed)) {
            state_frames_ = 0; sub_state_ = 1;
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
        }
        return;
    }
    if (sub_state_ == 1) {
        // Phase 2: 直走 20cm （在邻道上越过障碍正前方）
        if (walk_xy_distance(kObsFwdStep, 1.0f, 0.0f, face, kWalkSpeed)) {
            state_frames_ = 0; sub_state_ = 2;
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
        }
        return;
    }
    if (sub_state_ == 2) {
        // Phase 3: 45° 斜向回赛道（横移回 + 前进 同时进行）
        //   obstacle_side_=+1（先往右移） → 回赛道要左前 45°：(x+, y-) 即 (0.707, -0.707)
        //   obstacle_side_=-1（先往左移） → 回赛道要右前 45°：(x+, y+) 即 (0.707, +0.707)
        const float xd =  0.707107f;
        const float yd = -0.707107f * static_cast<float>(obstacle_side_);
        if (walk_xy_distance(kObsDiagDist, xd, yd, face, kObsDiagSpeed)) {
            state_frames_ = 0; sub_state_ = 3;
        }
        return;
    }
    if (sub_state_ == 3) {
        // Phase 4: 停止 + 复位标志 → 继续 NAVIGATE_CH 做任务
        clearing_after_handle_ = true;
        motion_.stop();
        ref_x_ = last_odom_x_ = sensor_.odom_x;
        ref_y_ = last_odom_y_ = sensor_.odom_y;
        travelled_since_ref_ = 0.0f;
        state_frames_ = 0; sub_state_ = 0;
        dashed_found_ = false; obstacle_side_ = 0;
        state_ = State::NAVIGATE_CH;
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════
// RETURN_ENTRANCE：原路返回通道入口
//   phase 0: 转180°（面向返回方向 = 通道朝向反方向）
//   phase 1: 边走边识别限高杆（遇到就蹲35cm过，返回时也要）
//            识别到尽头黄实线/走够total_travelled_兜底 → 到入口
//   phase 2: 转回 channel_yaw_ 入口朝向
//   phase 3: 通道3已完 → SEEK_BRIDGE；否则 → SWITCH_CHANNEL
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::return_entrance() {
    const float return_yaw = norm_yaw(channel_yaw_ + 3.14159f);  // 180°反向

    if (sub_state_ == 0) {
        if (turn_to_yaw(return_yaw)) {
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 1;
        }
        return;
    }
    if (sub_state_ == 1) {
        // 返回途中遇到限高杆 → 再过一遍
        if (sensor_.limbar_found && sensor_.limbar_dist < 1.2f && !crouch_active_) {
            target_yaw_ = return_yaw;
            crouch_active_ = true;
            motion_.set_step_height(kCrouchStepH, kCrouchStepH);
            state_frames_ = 0; sub_state_ = 10;
            return;
        }
        // 遇到黄实线边界 → 认为到入口了
        if (sensor_.divider_found && !sensor_.divider_is_dashed
            && sensor_.divider_dist > 0 && sensor_.divider_dist < kDividerStopDist) {
            motion_.stop();
            state_frames_ = 0; sub_state_ = 2;
            return;
        }
        // 兜底：走够了就到入口（=进入后走的 total_travelled_）
        if (travelled_since_ref_ >= total_travelled_ + 0.3f) {
            motion_.stop();
            state_frames_ = 0; sub_state_ = 2;
            return;
        }
        if (crouch_active_) {
            motion_.set_velocity(kCrawlSpeed, 0.0f, 0.0f);
        } else {
            motion_.set_velocity(kWalkSpeed, 0.0f, 0.0f);
        }
        return;
    }
    // 返回途中限高杆 sub_phase：蹲→走50cm→起
    if (sub_state_ == 10) {
        motion_.stop();
        if (++state_frames_ > kCrouchFrames
            && sensor_.body_height < kCrouchHeight + 0.03f) {
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 11;
        }
        return;
    }
    if (sub_state_ == 11) {
        if (walk_distance(kLimbarWalkDist, return_yaw, kCrawlSpeed)) {
            crouch_active_ = false;
            motion_.set_step_height(kStandStepH, kStandStepH);
            state_frames_ = 0; sub_state_ = 12;
        }
        return;
    }
    if (sub_state_ == 12) {
        motion_.stop();
        if (++state_frames_ > kCrouchFrames
            && sensor_.body_height > kStandHeight - 0.03f) {
            state_frames_ = 0; sub_state_ = 1;  // 继续返回
        }
        return;
    }
    if (sub_state_ == 2) {
        // 转回通道朝向
        if (turn_to_yaw(channel_yaw_)) {
            state_frames_ = 0; sub_state_ = 3;
        }
        return;
    }
    if (sub_state_ == 3) {
        in_channel_ = false;
        ++channel_count_;
        if (channel_count_ >= kMaxChannels) {
            // 最后一个通道走完 → 找桥
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 0;
            state_ = State::SEEK_BRIDGE;
        } else {
            // 还有通道 → 切换
            state_frames_ = 0; sub_state_ = 0;
            state_ = State::SWITCH_CHANNEL;
        }
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════
// SEEK_BRIDGE：找独木桥起始端 → 前腿足底碰到结束
// ═══════════════════════════════════════════════════════════════════
void Stage4Real::seek_bridge() {
    if (sensor_.tof_available && sensor_.tof_clearance < kBridgeClearance) {
        finish(); return;
    }
    if (sensor_.lidar_front > 0 && sensor_.lidar_front < kBridgeFrontDist) {
        finish(); return;
    }
    if (walk_distance(kMaxSeekBridgeDist, switch_yaw_, kWalkSpeed)) {
        finish();
    }
}
