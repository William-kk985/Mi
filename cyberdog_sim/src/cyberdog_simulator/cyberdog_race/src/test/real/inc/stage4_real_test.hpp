#pragma once
#include "cyberdog_race/stages/real/stage4_real.hpp"

// ★ Stage4 test 路线实现（运行期参数 stage4_impl:=test 选中，与正式版同二进制）
//   由运行期参数 stage4_impl:=test 选中 (launch/命令行), 与 Stage4Real(正式) 编在同一二进制。
//   test 路线: 起步1.8/1.1/1.2, 去程3.5/3.25/2.3, 回程3.6/3.5/3.6,
//              第3轮跳过右转→绕行(左转→1.1→右转→0.95→左转进通道),
//              第2轮结束左转衔接, 离场左转→0.8→直接左转立正。
//   只覆盖差异点(路线参数 + 3 个流转 hook), 其余逻辑复用基类 Stage4Real。
class Stage4RealTest : public Stage4Real {
public:
    using Stage4Real::Stage4Real;
    // init()/run()/is_done() 全部复用基类 (基类 init() 虚调用 set_route_params → 自动用本类参数)
protected:
    void set_route_params() override;      // test 路线距离/离场参数
    bool route_after_fwd1m() override;     // 第3轮起步走满 → 绕行
    bool route_after_round_out() override; // 第2轮结束 → 左转衔接
    bool route_after_exit_fwd1() override; // 离场第1段 → 直接左转立正
};
