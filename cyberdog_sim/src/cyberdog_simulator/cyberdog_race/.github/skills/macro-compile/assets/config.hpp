#pragma once

// ═════════════════════════════════════════════════════
// 集中式宏定义 — 所有编译时配置都在这里
// ═════════════════════════════════════════════════════

// ── 平台宏（互斥） ──
// #define PLATFORM_A
// #define PLATFORM_B

#if defined(PLATFORM_A) && defined(PLATFORM_B)
#error "PLATFORM_A and PLATFORM_B are mutually exclusive"
#endif

// ── 功能宏（注释=零开销） ──
// #define ENABLE_FEATURE_X
// #define ENABLE_FEATURE_Y

// ── 调试宏（发布前注释） ──
// #define DEBUG_VERBOSE

// ── 值宏 ──
// #define CONFIG_BUF_SIZE 256
