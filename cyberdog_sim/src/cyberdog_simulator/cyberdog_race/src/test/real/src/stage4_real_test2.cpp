#include "stage4_real_test2.hpp"
#include <cstdio>

// ═══════════════════════════════════════════════════════════════
// Stage4RealTest2 — test2 路线 (2026-09-02 重构: 原 DEBUG_STAGE4_TEST_ROUTE2 编译期宏迁移)
//   只覆盖与正式版的差异点, 其余复用基类 Stage4Real。
//   test2 路线动作: 起步0.8/1.2 → 去程3.5/3.35 → 回程3.6/3.6 (只跑2轮),
//   第1轮结束右转回正→前进1.8m衔接第2轮, 第2轮结束左转90°→前进2.5m→左转90°立正。
// ═══════════════════════════════════════════════════════════════
void Stage4RealTest2::set_route_params() {
    kFwdDist[0] = 0.8f; kFwdDist[1] = 1.2f; kFwdDist[2] = 1.0f;     // 起步 (轮1/2/3)
    kLaneDist[0] = 3.5f; kLaneDist[1] = 3.35f; kLaneDist[2] = 2.0f; // 去程通道
    kLaneDistBack[0] = 3.6f; kLaneDistBack[1] = 3.6f; kLaneDistBack[2] = 2.1f; // 回程通道
    // kExitFwd1/2/3: 保持基类正式值 (1.0/1.15/1.8) — test2 走正式四边形离场
    // kFwdLatComp/kFwdLatCompLow: 保持基类正式值 (0.023/0.020)
}

// 不绕行, 第3轮正常右转进通道
bool Stage4RealTest2::route_after_fwd1m() { return false; }

// 轮间衔接 (原 #ifdef DEBUG_STAGE4_TEST_ROUTE2)
bool Stage4RealTest2::route_after_round_out() {
    if (round_count_ == 0) {
        // 第1轮结束: 右转回正后前进1.8m衔接第2轮
        if (turn_to_yaw(entry_yaw_)) {
            link_yaw1_ = sensor_.abs_yaw;
            ++round_count_;
            fprintf(stderr, "\033[1;35m[S4T2] 第%d轮完成(前进1.8m衔接)\033[0m\n", round_count_);
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 0;
            tts_limbar_ = tts_coke_ = tts_football_ = tts_ball_ = tts_obstacle_ = false;
            scan_done_out_ = false;
            state_ = State::FWD_LINK_1;
        }
        return true;
    }
    if (round_count_ == 1) {
        // 第2轮结束: 左转90° → 前进2.5m → 左转立正
        if (turn_to_yaw(0.0f, +1.5708f)) {
            link_yaw2_ = sensor_.abs_yaw;
            ++round_count_;
            fprintf(stderr, "\033[1;35m[S4T2] 第%d轮完成(左转→2.5m→左转立正)\033[0m\n", round_count_);
            ref_x_ = last_odom_x_ = sensor_.odom_x;
            ref_y_ = last_odom_y_ = sensor_.odom_y;
            travelled_since_ref_ = 0.0f;
            state_frames_ = 0; sub_state_ = 0;
            tts_limbar_ = tts_coke_ = tts_football_ = tts_ball_ = tts_obstacle_ = false;
            scan_done_out_ = false;
            state_ = State::FWD_LINK_2;
        }
        return true;
    }
    return false;   // 其余轮次走基类右转回正
}

// 正式四边形离场
bool Stage4RealTest2::route_after_exit_fwd1() { return false; }
