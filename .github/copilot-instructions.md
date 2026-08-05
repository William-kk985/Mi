# cyberdog_race — 项目规范

## 核心架构：宏编译开关
所有平台/功能切换走 `debug_config.hpp`，严禁在 .cpp/.hpp 中硬编码。

| 宏 | 作用 |
|---|---|
| `REAL_DOG` | 定义=真机代码，注释=仿真代码 |
| `LLM_MODE_PROXY` / `LLM_MODE_API` | 二选一（有 #error 互斥保护） |
| `ENABLE_WEB_STREAMING` | 注释=零开销 |
| `DEBUG_*` | 发布前必须全注释掉 |

## 三层隔离

```
仿      真（virtual/）  →  Stage1~6       ← 独立类体系
真      机（real/）     →  Stage1Real~6Real ← 独立类体系，与仿真互不引用
测      试（test/）     →  Stage1RealTest~6RealTest ← 类名+Test后缀
测试编译开关            →  cmake -DUSE_TEST_REAL_STAGE3=ON 等
```

- 🚫 真机类**不得引用**仿真类，反之亦然
- 🚫 测试类**不得**污染正式代码目录
- ✅ 测试通过 cmake 条件编译控制，不污染正式版

## 线程安全红线
- 🚫 锁内操作 < 0.1ms
- ✅ `on_rgb`：锁外做 CV 检测+JPEG 编码，锁内仅写共享数据
- ✅ 读者自行加锁快照

## 构建命令
```bash
# 仿真（默认）
colcon build --merge-install --packages-select cyberdog_race
# 测试版
colcon build --cmake-args -DUSE_TEST_REAL_STAGE3=ON
# 真机版
# 取消 debug_config.hpp 中 // #define REAL_DOG 的注释后编译
```
