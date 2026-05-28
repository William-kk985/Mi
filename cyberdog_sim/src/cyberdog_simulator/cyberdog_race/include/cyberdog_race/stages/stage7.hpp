#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第七赛段：跳跃测试
// 向前跳，验证跳跃动作是否正确执行
class Stage7 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    bool is_done() override;
    bool needs_rc_mode() const { return rc_mode_needed_; }

private:
    bool done_{false};
    int   jump_frames_{0};
    float jump_start_x_{0.f}, jump_start_y_{0.f};
    bool  rc_mode_needed_{false};
    int   retry_count_{0};

    static constexpr float JUMP_TARGET = 0.6f;   // 跳跃目标距离60cm
    static constexpr float JUMP_MIN    = 0.04f;   // 模拟环境中至少4cm就算成功
    static constexpr int   MAX_RETRY  = 1;        // 跳1次+重试1次=共2次

    enum class State {
        START,        // 起立
        PRE_JUMP,     // 切换到RC模式，发送跳跃命令
        JUMP_FORWARD, // 等待跳跃完成
        CHECK_JUMP,   // 检查跳跃结果
        RESET,        // 退出跳跃模式准备重试
        FINISH        // 结束
    } state_{State::START};
};
