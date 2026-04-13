#include "cyberdog_race/stages/stage4.hpp"

// TODO: 第四赛段实现
// 依赖YOLO检测器，待集成后实现

void Stage4::init() {
    done_ = false;
    cur_channel_ = 0;
    coke_done_ = orange_ball_done_ = football_done_ = false;
    state_ = State::TRAVERSE;
    motion_.locomotion();
}

void Stage4::run() {
    if (done_) return;
    // TODO: 实现横向通道导航 + 竖向通道搜索 + 目标交互
}

bool Stage4::is_done() { return done_; }
