#pragma once
#include <deque>
#include <utility>
#include "cyberdog_race/stages/stage_base.hpp"

// 动作类型 (文件级, 供 cpp 定义动作序列)
enum class S2Kind { FWD, SLIDE_L, SLIDE_R, TURN_R90, TURN_L90, TURN_180, TURN_ABS };
struct S2Step { S2Kind kind; float dist; };   // TURN 步 dist 忽略; TURN_ABS 用 dist=角度(度, 正=左转)

/// 真机第2赛段 (2026-08-15 用户重定流程)
/// 每个左移/前进步出发前统一左转2° (2026-08-18 用户: 抵消越走越右偏; 3°太多/1°不转→折中2°)
/// 流程: 前进0.95 → 左移0.3 → 右转90° → 左移2.8 → 转180° → 左移2.8 →
///       前进1.2 → 右移2.7 → 左移0.15 → 前进1.12 → 左移2.8 → 右转90° → 左转4° → 前进3.3 →
///       前进0.1 → 左转90° → 前进0.5 → 右转90° → 前进0.6 → DONE
/// 移动中找正前方球撞击: 全场最多4球, 撞过的球位置防重, 撞击后原路退回
class Stage2Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { STEP, IMPACT, BACK, HOLD, FIX, DONE };

    void enter_impact(float ball_dist);   // 进入撞击: 记录起点/轨迹/球位置
    void do_step_fix();                   // 步末矫正: 航向→横向, 修完进下一步 (2026-08-16)
    void do_hold();                       // 撞击回点后站稳1s (2026-08-17 替代0.1m小步的恢复作用)

    Phase phase_{Phase::STEP};
    bool  done_{false};

    float start_abs_yaw_{0.0f};  // Stage2开始SLAM航向 (DONE诊断: 量化最终偏转, 2026-08-18)
    int   step_idx_{0};         // 当前动作步索引
    bool  step_start_{true};    // 步起点: 锁方向/重置累计
    float step_yaw_slam_{0.0f}; // 本步方向 (SLAM系, 位置投影用)
    float step_yaw_odom_{0.0f}; // 本步方向 (odom系, 航向锁用)
    float step_left_{0.0f};     // 本步剩余距离
    float step_start_x_{0.0f}, step_start_y_{0.0f};  // 本步起点 (2026-08-15 累计漂移闭环)
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};      // 本步累计位移
    float side_cmd_accum_{0.0f};   // 指令积分累计 (odom漏计兜底)
    float side_odom_accum_{0.0f};  // odom投影累计
    float yaw_integ_{0.0f};        // 航向锁积分项

    int   ball_confirm_{0};     // 球连续确认帧数
    float turn_target_{0.0f};   // 转向目标 (odom yaw)
    int   turn_settle_{0};      // 转向后停稳帧计数
    int   turn_guard_{0};       // 停稳复核补转帧计数 (2026-08-16 移植Stage1: 停稳后误差仍大→低速补转)
    float turn_start_x_{0.0f}, turn_start_y_{0.0f};  // 转向起点位置 (2026-08-16 ADJUST顶回基准)
    int   adjust_frames_{0};    // ADJUST顶回帧计数/超时 (2026-08-16)
    float drift_rate_{0.0f};    // 航向漂移率估计 rad/s (2026-08-16 yaw前馈, 跨步保留; 不加位置积分防SLAM放大)
    float last_yaw_err_{0.0f};  // 上一帧航向误差 (漂移率估计用)
    float fwd_integ_{0.0f};     // SLIDE步前向积分 (2026-08-16 闭环自动学习横移角偏, 平地SLAM可信)
    float last_dev_fwd_{0.0f};      // 上一帧dev_fwd (2026-08-18 撞击后odom假跳检测: 单帧跳>0.1m沿用上帧)
    float post_impact_guard_{0.0f}; // 撞击回点后保护剩余里程 (2026-08-18: 前0.5m内vx_lock限幅±0.05防左前冲)

    int   impact_lost_{0};      // 冲击中连续丢球帧数
    float impact_goal_{0.0f};   // 冲击兜底距离 = 确认球距
    int   touch_frames_{0};     // 触球连续帧计数
    float impact_x_{0.0f}, impact_y_{0.0f};   // 撞击起点 (回点目标)
    float impact_yaw_{0.0f};                  // 撞击前 SLAM 航向 (回点方向锁)
    int   back_timeout_{0};     // 回点超时帧计数

    // ── 步末矫正 FIX (2026-08-16 用户: 每小段走完矫正再继续) ──
    int   fix_phase_{0};        // 0=修航向 1=修横向
    int   fix_frames_{0};       // 超时兜底帧计数
    float fix_step_yaw_{0.0f};  // 本步机头航向 (slam系)
    float fix_step_dir_{0.0f};  // 本步移动方向 (世界系)
    float fix_step_dist_{0.0f}; // 本步距离
    float fix_lat_integ_{0.0f}; // 横向积分项 (2026-08-17 修得更干净)

    // ── 撞击回点后站稳 HOLD (2026-08-17 替代0.1m小步恢复作用) ──
    int   hold_frames_{0};      // 原地踏步帧计数

    // ── 段中锚点重置 (2026-08-17 替代0.1m小步分段作用: 每0.7m重锚, 误差不跨段累积) ──
    float anchor_x_{0.0f}, anchor_y_{0.0f};   // 当前漂移闭环锚点
    float anchor_traveled_{0.0f};             // 锚点里程

    int   hit_total_{0};        // 全场已撞球数 (上限4)
    std::deque<std::pair<float,float>> impact_path_;  // 撞击轨迹点~5cm (原路退回)
    std::deque<std::pair<float,float>> hit_spots_;    // 撞过的球世界位置 (防二次撞击)
};

