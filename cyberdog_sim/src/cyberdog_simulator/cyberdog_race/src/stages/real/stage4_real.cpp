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

// debug 用: 状态名
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
        case State::TURN_R_OUT:       return "TURN_R_OUT";        case State::TURN_SPEC_L:       return "TURN_SPEC_L";
        case State::FWD_SPEC_1:        return "FWD_SPEC_1";
        case State::TURN_SPEC_R:       return "TURN_SPEC_R";
        case State::FWD_SPEC_2:        return "FWD_SPEC_2";
        case State::TURN_SPEC_L2:      return "TURN_SPEC_L2";
        case State::FWD_LINK_1:        return "FWD_LINK_1";
        case State::FWD_LINK_2:        return "FWD_LINK_2";
        case State::TURN_LINK_L:       return "TURN_LINK_L";
        case State::TURN_L_FINAL:      return "TURN_L_FINAL";
        case State::FWD_EXIT_1:       return "FWD_EXIT_1";
        case State::TURN_EXIT_R:      return "TURN_EXIT_R";
        case State::FWD_EXIT_2:       return "FWD_EXIT_2";
        case State::TURN_EXIT_L:      return "TURN_EXIT_L";
        case State::FWD_EXIT_3:       return "FWD_EXIT_3";        case State::TURN_EXIT_L2:      return "TURN_EXIT_L2";        case State::RECOVERING:       return "RECOVERING";
        case State::DONE:             return "DONE";
    }
    return "?";
}

// ═══════════════════════════════════════════════════════════════
// init / 里程 / 感知
// ═══════════════════════════════════════════════════════════════
void Stage4Real::init() {
    set_route_params();   // 派生类可覆盖路线参数
    state_ = State::WAIT_FOR_SENSORS;
    done_ = false;
    odom_initialized_ = false;

    // ── 站起: 服务起来就会站立, 不主动干预 (避免已站立时被误判趴着→强站摔倒); 开跑前等 body_height≥0.23 ──
#ifdef DEBUG_STAGE
    fprintf(stderr, "[S4] init: body_h=%.2f (等待服务站起, 不主动干预)\n", sensor_.body_height);
    fflush(stderr);
#endif

    recovery_cmd_sent_ = false;
    gait_reengaged_ = false;
    recovery_attempts_ = 0;
    ready_frames_ = 0;
    state_frames_ = 0;
    sub_state_ = 0;
    round_count_ = 0;
    scan_done_out_ = false;   // 只去程停, 回程不停
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

    // 切换摔修复: set_pitch/stop 走 gamepad 通道真机不可靠, 改 303 静止命令
    motion_.select_gait(ServoGait::SLOW);
    motion_.set_step_height(kStandStepH, kStandStepH);
    motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);   // 303 静止抬平
    pitch_unlocked_ = false;
}

void Stage4Real::update_odometry() {
    // 识别停点静止时不计里程 (静止踏步 odom 会漂移, 计入会使去/回程距离不一致)
    if (state_ == State::SCAN_STOP) return;
    // 定位数据有值即累里程 (原 odom_valid 无人写恒 false 导致里程死)
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
                        && sensor_.abs_yaw != 0.0f   // 必须等航向就绪! 否则 entry_yaw=0→狗猛转圈摔倒
                        && sensor_.odom_x != 0.0f
                        && sensor_.body_height >= 0.23f   // 开跑前必须确认已站立, 防开始就摔
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
    if (std::abs(yaw_err) < kYawDeadband) yaw_err = 0.0f;   // 死区 (Stage2 同款)
    float yaw_cmd = clamp01(kYawKp * yaw_err, -kTurnRate, kTurnRate);
    if (travelled_since_ref_ >= distance) { motion_.stop(); return true; }
    // set_velocity 走 gamepad 通道会失控, 改 303; vy 打底 kFwdLatComp 补物理右偏
    motion_.set_walk_velocity_pitch(speed, kFwdLatComp, yaw_cmd, 0.0f);
    return false;
}

// 低姿行走: 用"低头前进"代替降身高 (201 降高实测会掉地)
//   303 + rpy_des[1]=kLimbarPitch(0.25≈14°)
//   ⚠ 必须先破限 x_effect_scale_pos=+30 (进段时设置), 否则步态硬限±5.7°会把命令夹掉
bool Stage4Real::walk_low(float distance, float target_yaw, float speed) {
    speed = clamp01(speed, -0.55f, 0.55f);
    float yaw_err = norm_yaw(target_yaw - sensor_.abs_yaw);
    if (std::abs(yaw_err) < kYawDeadband) yaw_err = 0.0f;   // 死区 (Stage2 同款)
    float yaw_cmd = clamp01(kYawKp * yaw_err, -kTurnRate, kTurnRate);
    if (travelled_since_ref_ - sub_start_travel_ >= distance) { motion_.stop(); return true; }
    // 破限段 vel_cmd_scale 被放大→横移补偿用更小 kFwdLatCompLow 防偏左过度
    motion_.set_walk_velocity_pitch(speed, kFwdLatCompLow, yaw_cmd, kLimbarPitch);
    return false;
}

bool Stage4Real::turn_to_yaw(float target_yaw, float rel_delta) {
    // 相对转向: 首帧快照当前 yaw+增量后锁定目标, 行走段 yaw 漂移不再影响增量
    if (rel_delta != 0.0f) {
        if (!turn_rel_valid_) {
            turn_rel_target_ = norm_yaw(sensor_.abs_yaw + rel_delta);
            turn_rel_valid_ = true;
        }
        target_yaw = turn_rel_target_;
    }
    target_yaw = norm_yaw(target_yaw + kTurnExtraRad + kTurnYawBias);
    // 原地转向漂移严重: 记录转向起点, 转向中横纵向拉回保持位置
    if (!turn_ref_valid_) {
        turn_ref_x_ = sensor_.odom_x;
        turn_ref_y_ = sensor_.odom_y;
        turn_ref_valid_ = true;
    }
    float yaw_err = norm_yaw(target_yaw - sensor_.abs_yaw);
    if (std::abs(yaw_err) <= kYawTol) {
        motion_.stop();
        turn_ref_valid_ = false;
        turn_rel_valid_ = false;
        return true;
    }
    float cmd = clamp01(std::abs(yaw_err) * kYawKp, 0.05f, kTurnRate);   // 接近目标减速收尾, 防惯性过冲"转多"
    // 位置保持: 世界偏差投影机体系, 限幅±kTurnHoldLimit 只对抗漂移不干扰转向; vy 固定左偏置补偿右漂
    const float dx = turn_ref_x_ - sensor_.odom_x;
    const float dy = turn_ref_y_ - sensor_.odom_y;
    const float a  = sensor_.abs_yaw;
    const float vx = clamp01(std::cos(a) * dx + std::sin(a) * dy, -kTurnHoldLimit, kTurnHoldLimit);
    const float vy = clamp01(-std::sin(a) * dx + std::cos(a) * dy + kTurnLatComp, -kTurnHoldLimit, kTurnHoldLimit);
    motion_.set_walk_velocity_pitch(vx, vy, yaw_err > 0.0f ? cmd : -cmd, 0.0f);
    return false;
}

bool Stage4Real::align_to_target(float cx, float tol) {
    if (std::abs(cx) <= tol) { motion_.stop(); return true; }
    float cmd = clamp01(std::abs(cx) * 0.6f, 0.08f, kTurnRate);
    // 方向: cx>0 偏右→右转(-cmd), cx<0 偏左→左转(+cmd) (反了会 KNOCK 对齐转圈)
    motion_.set_walk_velocity_pitch(0.0f, 0.0f, cx > 0.0f ? -cmd : cmd, 0.0f);
    return false;
}

// ═══════════════════════════════════════════════════════════════
// 恢复
// ═══════════════════════════════════════════════════════════════
void Stage4Real::enter_recovery() {
    if (state_ == State::RECOVERING || state_ == State::DONE) return;
    // 跌倒恢复前复原 pitch 限位, 防带着放大限位做恢复动作
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
        turn_ref_valid_ = false; turn_rel_valid_ = false;   // 恢复后转向基准重置
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
    motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);   // 303 静止 (不用 gamepad 通道)
    state_ = State::DONE; done_ = true;
}

bool Stage4Real::is_done() { return done_; }

// ═══════════════════════════════════════════════════════════════
// 2.8m 段全程低头: 进段破限 x_effect_scale_pos=+30, walk_low 带 kLimbarPitch 走全程; 出段恢复默认限位
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
    // 识别停点结束后固定前进 0.5m, 不再走剩余段
    const float lane_goal = post_scan_ ? kPostScanFwd : kLaneDist[round_count_];
    if (travelled_since_ref_ >= lane_goal) {
        post_scan_ = false;
        lane_pitch_restore();   // 离开 2.8m 段恢复限位
        motion_.stop();
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 去程走满%.2fm → 掉头\n", travelled_since_ref_); fflush(stderr);
#endif
        state_ = State::TURN_BACK;
        sub_state_ = 0; state_frames_ = 0;
        return;
    }

    // ── 识别停点: 剩 0.3m 处停 5 秒识别足球/可乐/橙球播报 ──
    if (!scan_done_out_ && travelled_since_ref_ >= kLaneDist[round_count_] - kScanStopMargin) {
        scan_done_out_ = true;
        scan_return_ = State::LANE_OUT;
        scan_frames_ = 0;
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 去程剩%.2fm → 识别停点5s\n", kLaneDist[round_count_] - travelled_since_ref_);
        fflush(stderr);
#endif
        state_ = State::SCAN_STOP;
        state_frames_ = 0;
        return;
    }

    const auto age = std::chrono::steady_clock::now() - last_vision_time_;
    if (age <= kVisionTimeout) {
        // 限高杆 <0.8m → 只播报 (全程已低头, 不再切低姿状态)
        if (sensor_.limbar_found && sensor_.limbar_dist > 0.0f
            && sensor_.limbar_dist < kLimbarTriggerDist) {
            if (!tts_limbar_) { motion_.speak("识别到限高杆"); tts_limbar_ = true; }
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S4] 去程限高杆 dist=%.2fm x=%.2f (全程低头, 直接过)\n",
                    sensor_.limbar_dist, sensor_.limbar_x); fflush(stderr);
#endif
        }
        // 可乐/足球/橙球只在停点识别, 走动中不识别不撞击不播报; 蓝色方块走路段一直识别
    } else {
        // 视觉超时：减速慢走（不平白停死）
        walk_low(kLaneDist[round_count_], lane_yaw_, kLowSpeed);
        return;
    }
    walk_low(kLaneDist[round_count_], lane_yaw_, kWalkSpeed);
}

// ═══════════════════════════════════════════════════════════════
// 限高杆：低姿（energy_saving → 身高0.13）前进1m → 起立
// ═══════════════════════════════════════════════════════════════
void Stage4Real::pass_limbar() {
    // 全程低头方案下不再进入此状态 (保留函数)
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
    // 低头方案收尾: 抬回头
    if (state_frames_ < kRiseFrames) {
        ++state_frames_;
        motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }
    // 过杆完成 → 恢复 pitch 限位默认值
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
        // 用实时球位置对齐 (knock_cx_ 快照永不满足 |cx|≤tol→原地转圈; 丢失/超时直接撞)
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
    const float back_dist = kLaneDistBack[round_count_];   // 回程按轮次取距
    if (travelled_since_ref_ >= back_dist) {
        lane_pitch_restore();   // 离开 2.8m 段恢复限位
        motion_.stop();
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 回程走满%.2fm → %s\n", travelled_since_ref_,
                (round_count_ + 1 >= kMaxRounds) ? "左转离场" : "右转出通道"); fflush(stderr);
#endif
        // 第 3 轮回程后左转离场, 不再右转出通道
        state_ = (round_count_ + 1 >= kMaxRounds) ? State::TURN_L_FINAL : State::TURN_R_OUT;
        sub_state_ = 0; state_frames_ = 0;
        return;
    }

    const auto age = std::chrono::steady_clock::now() - last_vision_time_;
    if (age <= kVisionTimeout
        && sensor_.limbar_found && sensor_.limbar_dist > 0.0f
        && sensor_.limbar_dist < kLimbarTriggerDist) {
        // 全程已低头, 只播报不切状态
        if (!tts_limbar_) { motion_.speak("识别到限高杆"); tts_limbar_ = true; }
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 回程限高杆 dist=%.2fm x=%.2f (全程低头, 直接过)\n",
                sensor_.limbar_dist, sensor_.limbar_x); fflush(stderr);
#endif
    }
    // 橙球只在停点识别, 回程走动中不播报; 蓝色方块走路段一直识别
    if (age > kVisionTimeout) {
        walk_low(back_dist, back_yaw_, kLowSpeed);
        return;
    }
    walk_low(back_dist, back_yaw_, kWalkSpeed);
}

// ═══════════════════════════════════════════════════════════
// 识别停点: 静止 5 秒播报可乐/足球/橙球, 再走完剩余 0.3m
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
    // 停点只识别可乐/足球/橙色球, 不识别蓝色块
    if (++scan_frames_ >= kScanHoldFrames) {
#ifdef DEBUG_STAGE
        fprintf(stderr, "[S4] 识别停点完成 → %s\n", state_name(scan_return_));
        fflush(stderr);
#endif
        // 识别完固定前进 0.5m 再掉头; 重置里程基准 (静止 odom 漂移不计入)
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
    // ── 心跳诊断: 每 20 帧打印, 看 control_loop 是否活着 ──
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
        // 切换摔修复: stop 走 gamepad 通道, 改 303 静止保持伺服模式
        motion_.set_walk_velocity_pitch(0.0f, 0.0f, 0.0f, 0.0f);
        update_perception();
        const bool ready = sensor_.rgb_valid
                        && sensor_.abs_yaw != 0.0f   // 必须等航向就绪
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

    // 蓝色障碍物走路段一直识别播报, 纯播报不影响运动逻辑; tts_obstacle_ 每轮复位
    if (sensor_.obstacle_found && !tts_obstacle_) {
        motion_.speak("识别到不可跨越障碍物");
        tts_obstacle_ = true;
    }

#ifdef DEBUG_STAGE
    // 状态切换日志
    if (state_ != last_log_state_) {
        last_log_state_ = state_;
        fprintf(stderr, "[S4] 状态→ %s (第%d轮 已走%.2f/%.2fm)\n",
                state_name(state_), round_count_ + 1,
                travelled_since_ref_,
                (state_ == State::LANE_OUT || state_ == State::LANE_BACK
                 || state_ == State::PASS_LIMBAR || state_ == State::KNOCK) ? kLaneDist[round_count_] : kFwdDist[round_count_]);
        fflush(stderr);
    }
#endif

    switch (state_) {
        case State::FWD_1M: {
            // 三轮起步分别走 1m/0.8m/1m
            if (walk_distance(kFwdDist[round_count_], entry_yaw_, kWalkSpeed)) {
                motion_.stop();
                state_frames_ = 0; sub_state_ = 0;
                // test 路线第 3 轮跳过右转→绕行, 由派生 hook 接管
                if (route_after_fwd1m()) return;
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
                lane_pitch_unlock();   // 进 2.8m 段全程低头
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::LANE_OUT;
            }
            return;
        }
        case State::TURN_SPEC_L: {   // test 路线: 第 3 轮左转 90° 绕行
            if (turn_to_yaw(0.0f, +1.5708f)) {
                spec_yaw1_ = sensor_.abs_yaw;
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::FWD_SPEC_1;
            }
            return;
        }
        case State::FWD_SPEC_1: {   // 前进1.0m
            if (walk_distance(kSpecialFwd1, spec_yaw1_, kWalkSpeed)) {
                motion_.stop();
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::TURN_SPEC_R;
            }
            return;
        }
        case State::TURN_SPEC_R: {   // 右转90°
            if (turn_to_yaw(0.0f, -1.5708f)) {
                spec_yaw2_ = sensor_.abs_yaw;
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::FWD_SPEC_2;
            }
            return;
        }
        case State::FWD_SPEC_2: {   // 前进1.1m
            if (walk_distance(kSpecialFwd2, spec_yaw2_, kWalkSpeed)) {
                motion_.stop();
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::TURN_SPEC_L2;
            }
            return;
        }
        case State::TURN_SPEC_L2: {   // 左转90°进通道
            if (turn_to_yaw(0.0f, +1.5708f)) {
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                sub_start_travel_ = 0.0f;
                lane_pitch_unlock();   // 进通道全程低头
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::LANE_OUT;
            }
            return;
        }
        case State::FWD_LINK_1: {   // test 路线2: 第 1 轮后前进 1.8m
            if (walk_distance(kLinkFwd1, link_yaw1_, kWalkSpeed)) {
                motion_.stop();
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::FWD_1M;
            }
            return;
        }
        case State::FWD_LINK_2: {   // test 路线2: 第 2 轮左转后前进 2.5m
            if (walk_distance(kLinkFwd2, link_yaw2_, kWalkSpeed)) {
                motion_.stop();
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::TURN_LINK_L;
            }
            return;
        }
        case State::TURN_LINK_L: {   // test 路线2: 最后左转 90° 立正
            if (turn_to_yaw(0.0f, +1.5708f)) {
                finish();
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
            // 180° 掉头转满 (用 back_yaw_ - kTurnExtraRad 抵消补偿)
            if (turn_to_yaw(back_yaw_ - kTurnExtraRad)) {
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                sub_start_travel_ = 0.0f;
                lane_pitch_unlock();   // 回程 2.8m 同样全程低头
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
            // test/test2 路线轮间衔接由派生 hook 接管, 正式版直接回正
            if (route_after_round_out()) return;
            // 前两轮出通道右转 90° 回正, 只有最后一轮才左转
            if (turn_to_yaw(entry_yaw_)) {
                ++round_count_;
                fprintf(stderr, "\033[1;35m[S4] 第%d轮完成\033[0m\n", round_count_);
                if (round_count_ >= kMaxRounds) { finish(); return; }
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                state_frames_ = 0; sub_state_ = 0;
                tts_limbar_ = tts_coke_ = tts_football_ = tts_ball_ = tts_obstacle_ = false;
                scan_done_out_ = false;   // 只去程停, 回程不停
                state_ = State::FWD_1M;
            }
            return;
        }
        case State::TURN_L_FINAL: {   // 第 3 轮回程后: 左转 90° 离场 (相对转向)
            if (turn_to_yaw(0.0f, +1.5708f)) {
                exit_yaw1_ = sensor_.abs_yaw;
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::FWD_EXIT_1;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S4] 左转完成, 离场前进%.1fm(不规则四边形)\n", kExitFwd1); fflush(stderr);
#endif
            }
            return;
        }
        case State::FWD_EXIT_1: {   // 不规则四边形离场: 左转 90° 后前进 1.0m
            if (walk_distance(kExitFwd1, exit_yaw1_, kWalkSpeed)) {
                motion_.stop();
                state_frames_ = 0; sub_state_ = 0;
                // test 路线 0.5m 后直接左转立正, 由派生 hook 决定
                state_ = route_after_exit_fwd1() ? State::TURN_EXIT_L2 : State::TURN_EXIT_R;
            }
            return;
        }
        case State::TURN_EXIT_R: {   // 右转 90° (相对转向)
            if (turn_to_yaw(0.0f, -1.5708f)) {
                exit_yaw2_ = sensor_.abs_yaw;
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::FWD_EXIT_2;
            }
            return;
        }
        case State::FWD_EXIT_2: {   // 前进1.0m
            if (walk_distance(kExitFwd2, exit_yaw2_, kWalkSpeed)) {
                motion_.stop();
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::TURN_EXIT_L;
            }
            return;
        }
        case State::TURN_EXIT_L: {   // 左转 90° (相对转向)
            if (turn_to_yaw(0.0f, +1.5708f)) {
                exit_yaw3_ = sensor_.abs_yaw;
                ref_x_ = last_odom_x_ = sensor_.odom_x;
                ref_y_ = last_odom_y_ = sensor_.odom_y;
                travelled_since_ref_ = 0.0f;
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::FWD_EXIT_3;
            }
            return;
        }
        case State::FWD_EXIT_3: {   // 前进1.5m → 再左转离场
            if (walk_distance(kExitFwd3, exit_yaw3_, kWalkSpeed)) {
                motion_.stop();
                state_frames_ = 0; sub_state_ = 0;
                state_ = State::TURN_EXIT_L2;
            }
            return;
        }
        case State::TURN_EXIT_L2: {   // 末尾再左转 90° 后立正
            if (turn_to_yaw(0.0f, +1.5708f)) {
                finish();
            }
            return;
        }
        default: return;
    }
}
