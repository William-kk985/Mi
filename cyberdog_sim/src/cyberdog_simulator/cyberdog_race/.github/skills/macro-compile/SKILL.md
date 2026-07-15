---
name: macro-compile
description: 'Create a new C++ project with macro-based compile-time configuration. Use when starting a new project from scratch that needs platform/feature switching via macros.'
---
# 宏编译骨架搭建

根据用户需求创建一个使用宏编译架构的新 C++ 项目。

## 步骤

1. 创建目录结构：`include/project/` `src/` 等
2. 创建 `config.hpp`（集中所有编译时宏）
3. 创建 `module_base.hpp`（模块基类）
4. 创建 `controller.hpp` + `controller.cpp`（主循环）
5. 创建 `CMakeLists.txt`（含条件编译选项）
6. 按需创建 `src/test/` 测试骨架

## 参考模板

- [config.hpp 模板](./assets/config.hpp)
- 完整规范见 README！.md
