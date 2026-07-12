# 真机赛段（待开发）

这些头文件定义了真机（REAL_DOG）模式下的赛段接口，但具体实现尚未开始。
仿真赛段（stages/virtual/）已完整实现，真机适配需基于此进行修改。

- `stage1_real.hpp` — 第1赛段真机版（直线行走）
- `stage2_real.hpp` — 第2赛段真机版（黄线循迹）
- `stage3_real.hpp` — 第3赛段真机版（找球+撞球）
- `stage4_real.hpp` — 第4赛段真机版（多目标分拣）
- `stage5_real.hpp` — 第5赛段真机版（独木桥）
- `stage6_real.hpp` — 第6赛段真机版（斜坡+翻转）

各赛段完全独立，互不干扰。参考 `stages/virtual/` 下对应赛段的实现完成 `.cpp`。
