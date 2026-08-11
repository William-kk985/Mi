#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

/// 赛段真机版 — 第2赛段
/// 右转3°→走1m→左转90°→走3.05m→右转90°
/// → 找球扫描: 左转45°停2秒 / 右转135°停2秒
///   每个角度: 识别到橙色球→前进0.2m再退回; 没有→2秒后转下一个角度
class Stage2Real : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override { return done_; }

private:
    enum class Phase { TURN1, FWD1, TURN2, FWD2, TURN3,
                       SCAN1_TURN, SCAN1_WAIT, SCAN1_ACT, SCAN1_BACK,
                       SCAN2_TURN, SCAN2_WAIT, SCAN2_ACT, SCAN2_BACK, DONE };

    Phase phase_{Phase::TURN1};
    bool  done_{false};
    int   turn_guard_{0};      // 转向最小帧数保护
    int   wait_frames_{0};     // 扫描停2秒计数
    float turn_base_yaw_{0.0f};   // 进入转向时的朝向 (相对当前转)
    float fwd_ref_yaw_{0.0f};     // 前进段目标朝向 (回正基准)
    float start_yaw_{0.0f};       // 赛段初始朝向
    float last_x_{0.0f}, last_y_{0.0f};
    float traveled_{0.0f};        // 累计位移 (前进/扫描戳球)
};
