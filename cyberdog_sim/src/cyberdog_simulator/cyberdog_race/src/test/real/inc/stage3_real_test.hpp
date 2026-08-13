#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 测试版 Stage3 — 伙伴连通域寻线算法实验 (2026-08-14)
/// 视觉: LaneDetector v2 (test/real, 连通域轨迹跟踪 + lookahead 采样)
///   ⚠ 伙伴版 curvature 是无符号方差, 不能做方向前馈 → 本测试版不用曲率项
/// 控制: yaw = clamp(-KP*off - 微分限幅, ±YAW_LIM), 丢线保持转向衰减
/// 编译: colcon build --cmake-args -DUSE_TEST_REAL_STAGE3=ON
/// ⚠ 测试版不污染正式代码目录, 默认编译不包含
class Stage3RealTest : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { WAIT_READY, LANE_FOLLOW, DONE };

    Phase phase_{Phase::WAIT_READY};
    bool  done_{false};
    bool  loc_ready_{false};   // 定位就绪 (spin后才有数据, 同Stage1)
    int   pitch_hold_{0};      // 201低头发布计数 (测试形态用)
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};     // 巡线累计位移
    float last_offset_{0.0f};  // 上帧 lane_offset (微分预测项)
    float last_yaw_{0.0f};     // 丢线保持的最后转向 (赛道出画面时继续转拉回)
};
