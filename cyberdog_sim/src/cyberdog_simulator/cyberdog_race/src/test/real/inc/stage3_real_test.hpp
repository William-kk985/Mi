#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

class Stage3RealTest : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override;
};
