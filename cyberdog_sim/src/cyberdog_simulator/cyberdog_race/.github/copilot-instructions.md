# cyberdog_race — 项目规范

## 架构
- 主循环 100Hz，赛段多态调度（`stages_[cur_stage_]->run()`）
- `on_rgb` 锁外做检测，锁内仅写共享数据（<0.01ms）
- 所有检测器统一接收 BGR，cv_bridge 自动转换源编码
- 鱼眼/深度/D430i 红外回调只做 Web 展示，不做检测

## 编译宏体系（debug_config.hpp）
- `REAL_DOG` 定义=真机，注释=仿真
- `LLM_MODE_PROXY` / `LLM_MODE_API` 二选一（#error 互斥保护）
- `ENABLE_WEB_STREAMING` 注释=零开销
- 调试宏（`DEBUG_*`）发布前必须注释

## 关键约束
- 🚫 不要在 .cpp/.hpp 硬编码 topic 名，统一走 debug_config.hpp
- 🚫 锁内操作 < 0.1ms
- 🚫 真机赛段类（StageNReal）与仿真赛段类（StageN）互不引用
- ✅ 新增 .cpp 必须加 CMakeLists.txt SOURCES
- ✅ 测试代码放 src/test/，通过 cmake -DUSE_TEST_XXX=ON 控制

## 构建命令
```bash
# 仿真
colcon build --merge-install --packages-select cyberdog_race
# 测试版
colcon build --cmake-args -DUSE_TEST_REAL_STAGE3=ON
```
