#pragma once
#include "cyberdog_race/stages/stage_base.hpp"
#include <array>

// 第二赛段：荒野寻珠
// S型扫描4×4球阵，撞击橙色球，走到左上角出口
class Stage2 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    bool is_done() override;

private:
    // 球阵状态：true=已处理（橙色已撞/蓝色跳过）
    std::array<std::array<bool, 4>, 4> ball_done_{};
    int cur_row_{3};   // 当前行（0=R1, 3=R4），从R4开始
    int cur_col_{3};   // 当前列
    bool going_left_{true};  // 当前行扫描方向
    bool done_{false};

    enum class State { MOVING, DETECTING, HITTING, TO_EXIT } state_{State::MOVING};
};
