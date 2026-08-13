#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第2赛段 (2026-08-13 四轮S形)
/// 衔接: 前进0.92m→左转90° (Stage1结束后进入赛道)
/// 轮1: 前进2.8m→右转90°→左右扫描→回正→前进0.75m
/// 轮2: 右转90°→前进2.8m→左转90°→左右扫描→回正→前进0.75m
/// 轮3: 左转90°→前进2.8m→右转90°→左右扫描→回正→前进0.75m
/// 轮4: 右转90°→前进2.8m→左转90°→左右扫描→结束
class Stage2Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { FWD0, TURN0, TURN1, FWD1, TURN2,
                       SCAN1_TURN, SCAN1_WAIT, SCAN1_ACT, SCAN1_BACK,
                       SCAN2_TURN, SCAN2_WAIT, SCAN2_ACT, SCAN2_BACK,
                       TURN3, FWD3, ADJUST, DONE };

    Phase phase_{Phase::FWD0};
    Phase after_adjust_{Phase::DONE};  // ADJUST 修正完成后去向 (2026-08-13)
    bool  done_{false};
    int   round_{0};           // 轮次 0/1/2 (2026-08-13 三轮S形)
    int   turn_guard_{0};      // 转向最小帧数保护
    int   turn_settle_{0};     // 转向停稳确认帧 (2026-08-12 提高转向精度)
    int   wait_frames_{0};     // 扫描停2秒计数
    int   adjust_frames_{0};   // ADJUST 修正帧计数/超时保护 (2026-08-13)
    int   ball_confirm_{0};    // 球连续确认帧数 (2026-08-13 防误检)
    float turn_base_yaw_{0.0f};   // 进入转向时的朝向 (相对当前转)
    float fwd_ref_yaw_{0.0f};     // 前进段目标朝向 (回正基准)
    float start_yaw_{0.0f};       // 赛段初始朝向
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};        // 累计位移 (前进/扫描戳球)
    float hold_x_{0.0f}, hold_y_{0.0f};            // 原地位置保持基准 (2026-08-13 防踏步后退)
    float turn_start_x_{0.0f}, turn_start_y_{0.0f}; // 转向起点 (2026-08-13 转向后顶回后退漂移)
};
