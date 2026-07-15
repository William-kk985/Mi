---
name: macro-compile
description: 'Create a new C++ project with macro-based compile-time configuration. Use when starting a new project from scratch that needs platform/feature switching via macros. Complete guide with naming conventions, thread safety rules, and file organization templates.'
---

# 宏编译骨架架构规范

> 本文档定义一种通用的 C++ 项目架构模式：通过预处理宏实现编译时配置。
> 无论是什么领域（机器人、嵌入式、服务端），只要你想 **一套代码 → 多套编译产物**，这个模式都适用。

---

## 核心思想

用一个 **集中式宏头文件** 来控制所有编译时行为：

```
一个代码库  →  宏开关  →  多个编译目标

    ┌─ config.hpp               # ★ 所有宏集中定义（唯一入口）
    │
    ├── 平台宏  →  #define PLATFORM_XXX  →  平台相关代码
    ├── 功能宏  →  #define ENABLE_XXX     →  特性开/关
    ├── 调试宏  →  #define DEBUG_XXX      →  诊断/日志/可视化
    └── 值宏    →  #define CONFIG_XXX N    →  编译期常量

特点：
· 零开销抽象 — 注释掉的宏对应代码不参与编译
· 单一入口 — 所有宏写在 config.hpp，.cpp/.hpp 不硬编码
· 互斥保护 — 冲突宏有 #error 编译期检查
· 枚举可查 — 看一眼 config.hpp 就知道项目支持什么
```

### 典型用法

```cpp
// config.hpp — 所有宏集中在这里

// ── 平台宏（三选一） ──
// #define PLATFORM_A
// #define PLATFORM_B
// #define PLATFORM_C

#if defined(PLATFORM_A) && defined(PLATFORM_B)
#error "PLATFORM_A and PLATFORM_B are mutually exclusive"
#endif

// ── 功能宏（注释 = 零开销） ──
// #define ENABLE_LOGGING
// #define ENABLE_VISUALIZATION
// #define ENABLE_EXTRA_MODULE

// ── 调试宏（发布前全注释掉） ──
// #define DEBUG_VERBOSE
// #define DEBUG_DRAW

// ── 值宏 ──
// #define CONFIG_BUF_SIZE 256
```

---

## 整体架构

```
controller (主循环)
  │
  ├─ 数据回调（只写共享数据，耗时操作放锁外）
  │     ↓
  ├─ shared_data_ (线程安全共享数据 POD，互斥锁保护)
  │
  ├─ 主循环
  │     ├─ modules_[current]->run()       ← 多态调度
  │     ├─ apply_config()                 ← 参数统一下发
  │     └─ 模块切换逻辑
  │
  └─ 可选服务（功能宏控制编译）
        ├─ 日志
        ├─ 可视化
        └─ ...
```

### 数据流通用原则

| 原则 | 说明 | 违反后果 |
|---|---|---|
| 回调只写锁 | 回调中耗时操作放锁外 | 阻塞其他回调 |
| 读者自行快照 | 读者在锁内拷贝所需字段后立即解锁 | 数据竞争 |
| 锁内 < 0.1ms | 锁内只做简单赋值 | 抢占优先级反转 |
| 可选服务隔离 | 可选服务不得在核心逻辑中硬引用 | 编译耦合 |

---

## 宏命名规范

| 类别 | 前缀 | 示例 | 说明 |
|---|---|---|---|
| 平台/目标 | `PLATFORM_` | `PLATFORM_LINUX`, `PLATFORM_EMBEDDED` | 互斥，三选一 |
| 功能开关 | `ENABLE_` | `ENABLE_LOGGING`, `ENABLE_VIS` | 注释=零开销 |
| 调试开关 | `DEBUG_` | `DEBUG_VERBOSE`, `DEBUG_DRAW` | 发布前全注释 |
| 值/参数 | `CONFIG_` | `CONFIG_BUF_SIZE 256` | 编译期常量 |
| 版本信息 | `VERSION_` | `VERSION_MAJOR 1` | 构建信息 |
| 依赖检测 | `HAS_` | `HAS_OPENCV`, `HAS_CURL` | cmake 传递 |

### 命名注意事项

- **不要用无前缀的短宏名**（如 `#define DEBUG`），容易和系统宏冲突
- **值宏用全大写 + 下划线**，和函数/变量区分
- **功能宏语义用肯定形式**：`ENABLE_XXX` 而非 `DISABLE_XXX`
- **互斥宏组**在 `config.hpp` 中用 `#error` 保护，且同一组用相同前缀

---

## 哪些该用宏，哪些不该

### ✅ 适合用宏的

| 场景 | 理由 |
|---|---|
| 平台/硬件差异 | 不同平台代码不兼容，编译期固定 |
| 发布 vs 调试 | 调试代码（日志、断言）发布时不应存在 |
| 可选第三方库 | 没有库时相关代码不编译 |
| 性能敏感的小常量 | 循环次数、缓冲区大小 |
| 断言/安全检查 | `assert`、边界检查在 Release 去掉 |

### ❌ 不适合用宏的

| 场景 | 理由 | 替代方案 |
|---|---|---|
| 运行时可配置的参数 | 修改参数需要重新编译 | 配置文件 / 命令行参数 |
| 复杂类型 / 类定义 | 宏无法提供类型安全 | `typedef` / `using` |
| 超过 10 行的代码块 | 难调试、难阅读 | 普通函数 / 模板 |
| 需要作用域隔离 | 宏不认 `{}` | `constexpr` / `inline` 函数 |
| 有副作用的表达式 | `MAX(a++, b)` 多次求值 | `std::max` 模板 |

---

## 安全注意事项

### 宏展开副作用

```cpp
// ❌ 危险：参数被多次求值
#define SQUARE(x) ((x) * (x))
int y = SQUARE(++x);  // → x 自增两次

// ✅ 安全：用内联函数
inline int square(int x) { return x * x; }
```

### include 顺序

```cpp
// 推荐顺序（每类内部按字母序）
#include "project/config.hpp"     // 1. 本项目的 config（最先，影响后续 include）

#include <opencv2/opencv.hpp>    // 2. 第三方库
#include <curl/curl.h>

#include <memory>                 // 3. C++ 标准库
#include <string>

#include "project/controller.hpp" // 4. 本项目其他头文件
#include "project/module_base.hpp"
```

- `config.hpp` 必须是第一个 include，因为它定义的宏会影响后续头文件的行为
- 标准库放在第三方库后面，本项目头文件最后

### #pragma once  vs  #define guard

- **统一用 `#pragma once`**，简洁、防重名
- 如果编译器不支持（极少数交叉编译工具链），回退到传统 `#ifndef` guard

---

## 文件组织规范

```
project/
├── CMakeLists.txt              # 编译配置（新增 .cpp 必须加 SOURCES）
├── agent.md                    # ← 本文档
│
├── include/project/
│   ├── config.hpp              # ★ 所有编译时宏
│   ├── controller.hpp          # 主控制器声明
│   ├── module_base.hpp         # 模块基类（纯虚接口）
│   │
│   ├── modules/                # 各平台/场景的模块实现
│   │   ├── impl_a/             # 实现 A 版
│   │   └── impl_b/             # 实现 B 版（独立类，互不引用）
│   │
│   ├── utils/                  # 可选工具（功能宏控制编译）
│   └── shared/                 # 共享数据定义
│       └── data.hpp
│
└── src/
    ├── main.cpp
    ├── controller.cpp
    ├── modules/impl_a/         # 实现 A 的 .cpp
    ├── modules/impl_b/         # 实现 B 的 .cpp
    └── test/                   # 测试代码目录（cmake 条件编译）
        ├── function/           # 通用测试函数
        │   ├── inc/
        │   │   └── behavior_test.hpp
        │   └── src/
        │       └── behavior_test.cpp
        ├── real/               # 真机测试
        │   ├── inc/
        │   │   └── moduleN_test.hpp
        │   └── src/
        │       └── moduleN_test.cpp
        └── virtual/            # 虚拟测试
            ├── inc/
            └── src/
```

---

## 关键约定（不得违反）

### 1. 宏体系 — config.hpp

- 平台宏控制编译目标，**不允许**在 .cpp 中硬编码平台相关值
- 功能宏控制特性编译，注释掉=零开销
- 调试宏发布前必须注释掉
- 互斥宏组合必须有 `#error` 编译期检查

```cpp
#if defined(MODE_A) && defined(MODE_B)
#error "MODE_A and MODE_B are mutually exclusive"
#endif
```

### 2. 线程安全

| 数据 | 写者 | 读者 | 保护方式 |
|---|---|---|---|
| shared_data_ 字段 | 回调线程 | 主循环 / 渲染 | `std::mutex` |
| 可选服务缓冲 | 任意 | 独立服务线程 | 独立锁 + condition_variable |
| 历史数据 | 主循环 | 渲染 | 锁内快照 |

**黄金法则**：锁内操作 < 0.1ms。任何耗时操作必须放锁外。

### 3. 实现隔离

| 层 | impl_a/ | impl_b/ |
|---|---|---|
| 模块逻辑 | 继承统一基类，完整实现 | 独立类，互不引用 |
| 资源路径 | config.hpp 中宏值 | config.hpp 中宏值 |
| 通信方式 | 平台相关实现 | 平台相关实现 |

不同实现的类名加不同后缀（如 `ModuleA` / `ModuleB`），完全独立。

### 4. 可选服务

- 所有可选服务包裹在 `#ifdef ENABLE_XXX` 中
- 服务类在头文件声明，实现只在宏启用时编译
- **不得**在核心控制逻辑中直接引用可选服务的类型

### 5. 新增 .cpp 文件

必须在 `CMakeLists.txt` 的 `set(SOURCES ...)` 中添加，否则链接错误。

### 6. 测试文件隔离

- 测试代码统一放在 `src/test/` 下，与正式代码完全分离
- 每个平台（`real/`、`virtual/`）各自维护 `inc/` + `src/`，自包含
- 测试通过 cmake 条件编译（`-DUSE_TEST_XXX=ON`）控制，不污染正式版
- 测试类名加 `Test` 后缀（如 `Stage1RealTest`），与正式实现完全独立

---

## 如何新增一个模块

1. `modules/moduleN.hpp` 继承 `ModuleBase`，实现 `init/run/is_done`
2. `modules/moduleN.cpp` 实现
3. 控制器 include 头文件 + 注册 `modules_[N] = std::make_unique<ModuleN>(...)`
4. 不同平台版：`modules/impl_b/moduleN_implB.hpp` 独立类
5. `CMakeLists.txt` SOURCES 添加 `.cpp`

---

## 编译

```bash
# 默认
mkdir build && cd build && cmake .. && make -j

# 打开某个宏：取消 config.hpp 中注释，或 cmake -DMACRO=ON
```

---

## 完整文档

参见 `开发状态！.md` 获取当前项目进度。
