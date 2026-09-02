#pragma once
#include "cyberdog_race/stages/real/stage4_real.hpp"

// ★ Stage4 test2 路线实现 (2026-09-02 重构: 原 DEBUG_STAGE4_TEST_ROUTE2 编译期宏迁移)
//   由运行期参数 stage4_impl:=test2 选中 (launch/命令行), 与 Stage4Real/Stage4RealTest 编在同一二进制。
//   test2 路线: 起步0.8/1.2, 去程3.5/3.35, 回程3.6/3.6 (只跑2轮 + 链接段),
//              第1轮结束右转回正→前进1.8m衔接, 第2轮结束左转→前进2.5m→左转立正。
//   只覆盖差异点(路线参数 + route_after_round_out 链接流转), 其余复用基类 Stage4Real。
class Stage4RealTest2 : public Stage4Real {
public:
    using Stage4Real::Stage4Real;
protected:
    void set_route_params() override;      // test2 路线距离参数
    bool route_after_fwd1m() override;     // 不绕行, 返回 false
    bool route_after_round_out() override; // 轮间链接段衔接
    bool route_after_exit_fwd1() override; // 正式四边形离场, 返回 false
};
