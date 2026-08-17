#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 测试版 Stage3 — 伙伴连通域寻线算法实验 (2026-08-14)
/// 视觉: LaneDetector v2 (test/real, 连通域轨迹跟踪 + lookahead 采样)
/// 控制: 伙伴版 (降速+丢线减速搜索+低通微分, 2026-08-14)
///   WALK_V=0.26 窄视野降速; 丢线<20帧保持转向衰减, ≥20帧停前进原地搜索
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
    float filtered_d_offset_{0.0f};  // 逐帧偏差变化低通 (伙伴版阻尼)
    int   lost_frames_{0};           // 连续丢线帧数 (伙伴版搜索状态机)
    bool  single_locked_{false};     // 单线模式: 已锁线位置 (2026-08-14 沿线趋势走)
    float lock_line_x_{0.0f};        // 锁定的线横向位置(归一化) (2026-08-14)
};
