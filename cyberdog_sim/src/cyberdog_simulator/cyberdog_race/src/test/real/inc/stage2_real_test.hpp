#pragma once
#include <deque>
#include <utility>
#include "cyberdog_race/stages/stage_base.hpp"

// 动作类型 (文件级, 供 cpp 定义动作序列)
enum class S2TestKind { FWD, SLIDE_L, SLIDE_R, TURN_R90, TURN_L90, TURN_180 };
struct S2TestStep { S2TestKind kind; float dist; };   // TURN 步 dist 忽略

/// 真机第2赛段 (2026-08-16 点位全局闭环版)
/// 动作序列与折线版一致(20步), 但每步导航到"全局目标点":
///   · 目标点从起点+理论航向递推(坐标系开机随机无所谓, 相对几何一致)
///   · 移动步: 沿步方向走 + 垂直到全局目标线闭环, 距目标点≤0.05m 到位
///   · 转向步: odom相对增量(理论航向同步更新)
///   · 撞击中断→原路退回→继续导航同一步(目标点不变, 天然兼容)
///   · 漂移不跨步累积: 每步都收敛到绝对目标点
class Stage2RealTest : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { STEP, IMPACT, BACK, DONE };

    void enter_impact(float ball_dist);   // 进入撞击: 记录起点/轨迹/球位置

    Phase phase_{Phase::STEP};
    bool  done_{false};
    bool  loc_ready_{false};   // 定位就绪 (odom首帧, 2026-08-16)

    int   step_idx_{0};        // 当前动作步索引
    bool  step_start_{true};   // 步起点: 推算目标点/重置
    float yaw_slam_{0.0f};     // 理论航向(SLAM系, 目标点推算, 只按指令更新防SLAM跳变)
    float yaw_odom_{0.0f};     // 理论航向(odom系, 转向闭环+航向锁)
    float target_x_{0.0f}, target_y_{0.0f};   // 本步全局目标点
    float turn_target_{0.0f};  // 转向目标 (odom yaw)
    int   turn_settle_{0};     // 转向后停稳帧计数
    int   step_timeout_{0};    // 步超时帧计数 (3000帧=30s防卡)
    int   step_time_{0};       // 本步已走帧数 (2026-08-16 到位判定指令积分下限, 防odom虚大早停)

    int   ball_confirm_{0};    // 球连续确认帧数
    int   impact_lost_{0};     // 冲击中连续丢球帧数
    float impact_goal_{0.0f};  // 冲击兜底距离 = 确认球距
    int   touch_frames_{0};    // 触球连续帧计数
    float impact_x_{0.0f}, impact_y_{0.0f};   // 撞击起点 (回点目标)
    float impact_yaw_{0.0f};                 // 撞击前 SLAM 航向 (回点方向锁)
    int   back_timeout_{0};    // 回点超时帧计数
    float last_x_{0.0f}, last_y_{0.0f};      // 撞击位移累计
    float traveled_{0.0f};     // 撞击累计位移
    float yaw_integ_{0.0f};    // 航向锁积分项

    int   hit_total_{0};       // 全场已撞球数 (上限4)
    std::deque<std::pair<float,float>> impact_path_;  // 撞击轨迹点~5cm (原路退回)
    std::deque<std::pair<float,float>> hit_spots_;    // 撞过的球世界位置 (防二次撞击)
};
