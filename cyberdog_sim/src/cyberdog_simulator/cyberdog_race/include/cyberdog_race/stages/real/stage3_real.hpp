#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第3赛段: 视觉巡线 (两边黄色赛道, 小弧线/直线/弧线)
/// 纯视觉PD控制: 直道提速 + 弯道减速 + 丢线保持 (2026-08-12)
/// 检测: LaneDetector 黄色赛道中线 (on_rgb, /image RGB摄像头)
/// 真机注意: IMU yaw 恒0, 回正只靠视觉 offset
class Stage3Real : public StageBase {
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
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};     // 累计位移 (防定位跳变)
    float prev_offset_{0.0f};  // 上一帧偏移 (微分项)
    float last_yaw_cmd_{0.0f}; // 丢线时保持
    int   lane_lost_{0};       // 丢线帧计数
};
