#include "stage4_real_test.hpp"
#include <cstdio>

// ═══════════════════════════════════════════════════════════════
// Stage4RealTest — test 路线（只覆盖与正式版的差异点，其余复用基类 Stage4Real）
//   只覆盖与正式版的差异点, 其余复用基类 Stage4Real。
//   test 路线动作: 起步1.8/1.1/1.2 → 去程3.5/3.25/2.3 → 回程3.6/3.5/3.6,
//   第3轮跳过右转直接绕行(左转90°→1.1m→右转90°→0.95m→左转90°进通道),
//   第2轮结束左转90°衔接第3轮, 离场左转90°→0.8m→直接左转90°立正。
// ═══════════════════════════════════════════════════════════════
void Stage4RealTest::set_route_params() {
    kFwdDist[0] = 1.8f; kFwdDist[1] = 1.1f; kFwdDist[2] = 1.2f;    // 起步 (轮1/2/3)
    kLaneDist[0] = 3.5f; kLaneDist[1] = 3.25f; kLaneDist[2] = 2.3f; // 去程通道
    kLaneDistBack[0] = 3.6f; kLaneDistBack[1] = 3.5f; kLaneDistBack[2] = 3.6f; // 回程通道
    kExitFwd1 = 0.8f;   // 离场: 左转90°后前进0.8m, 再左转90°立正
    kExitFwd2 = 1.15f;  // test 不用
    kExitFwd3 = 1.5f;   // test 不用
    // kFwdLatComp/kFwdLatCompLow: 保持基类正式值 (0.023/0.020)
}

// 第3轮起步走满 → 跳过右转，直接绕行左转
bool Stage4RealTest::route_after_fwd1m() {
    if (round_count_ == 2) { state_ = State::TURN_SPEC_L; return true; }
    return false;
}

// 第2轮结束(TURN_R_OUT) → 左转90°衔接第3轮
bool Stage4RealTest::route_after_round_out() {
    if (round_count_ != 1) return false;   // 其余轮次走基类右转回正
    if (turn_to_yaw(0.0f, +1.5708f)) {
        entry_yaw_ = sensor_.abs_yaw;   // 更新第3轮起步朝向; lane_yaw_/back_yaw_不动
        ++round_count_;
        fprintf(stderr, "\033[1;35m[S4Test] 第%d轮完成(左转衔接)\033[0m\n", round_count_);
        ref_x_ = last_odom_x_ = sensor_.odom_x;
        ref_y_ = last_odom_y_ = sensor_.odom_y;
        travelled_since_ref_ = 0.0f;
        state_frames_ = 0; sub_state_ = 0;
        tts_limbar_ = tts_coke_ = tts_football_ = tts_ball_ = tts_obstacle_ = false;
        scan_done_out_ = false;
        state_ = State::FWD_1M;
    }
    return true;   // 已接管 (转完前也由本 hook 继续处理)
}

// 离场第1段走满 → 直接左转立正
bool Stage4RealTest::route_after_exit_fwd1() { return true; }
