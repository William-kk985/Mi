#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

class Stage1RealTest : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override;
};
