# LLM 大模型集成使用指南

## 当前状态

LLM 助手层已完成（`llm_helper.hpp/cpp`），ROS2 Service 通信已联通，**但尚未接入任何赛段的决策逻辑中**。

## 通信架构

```
┌─────────────┐   ROS2 Service    ┌──────────────┐   HTTPS    ┌──────────┐
│ CyberDog    │  /llm_ask ──────► │ llm_bridge.py│ ────────► │ DeepSeek │
│ (PROXY模式) │ ◄── response ──── │  (笔记本)     │ ◄──────── │ /OpenAI  │
└─────────────┘                   └──────────────┘           └──────────┘
```

## 启用方式

1. 在 `debug_config.hpp` 中启用：
```cpp
#define LLM_MODE_PROXY   // ROS2 Service 模式（推荐比赛用）
// #define LLM_MODE_API   // libcurl 直连（无需笔记本中转）
```

2. 笔记本端启动桥接节点：
```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
export LLM_API_KEY="sk-your-key-here"
export LLM_MODEL="deepseek-chat"
python3 tools/llm_bridge.py
```

3. 启动狗端控制器，日志中出现 `[LLM] PROXY connected to /llm_ask` 表示联通。

## API 接口（LLMHelper 类）

```cpp
bool init(rclcpp::Node* node);                   // PROXY: 传入 this, API: 可不传
std::string ask(const std::string& prompt, int timeout_sec = 3);
bool ping();                                      // 检查连接是否存活
```

## 接入赛段的方法

### 方案一：赛段决策辅助（推荐）

在需要 AI 决策的赛段 `run()` 中调用 LLM：

```cpp
// Stage4::run() — 障碍判断示例
#ifdef LLM_MODE_PROXY
if (obstacle_detected && !llm_asked_) {
    std::string prompt = cv::format(
        "前方检测到障碍物，距离%.1fm，yaw=%.1f°，"
        "请选择: left/right/stop/jump。只回复JSON。",
        obstacle_dist, sensor_.yaw);
    std::string reply = llm_->ask(prompt, 3);
    if (!reply.empty()) {
        // 解析 {"action":"left","speed":0.3}
        llm_asked_ = true;
    }
}
#endif
```

### 方案二：全局异常处理

在 `control_loop()` 中，当连续多帧检测异常时询问 LLM：

```cpp
if (stuck_frames > 100) {
    std::string reply = llm_.ask("机器人卡住了，前方lidar=" + ...);
    // 解析回复执行脱困动作
}
```

### 方案三：赛段间过渡决策

在赛段切换时调用 LLM 做高级决策：

```cpp
if (stages_[cur_stage_]->is_done()) {
    // 询问 LLM 确认可以进入下一赛段
    std::string reply = llm_.ask("Stage" + std::to_string(cur_stage_+1)
        + "完成，是否继续Stage" + std::to_string(cur_stage_+2) + "?");
}
```

## 注意事项

| 事项 | 说明 |
|---|---|
| **超时设置** | `LLM_TIMEOUT=3` 秒，不要设置过长以免阻塞控制循环 |
| **阻塞问题** | `llm_.ask()` 是同步阻塞调用，建议在非关键路径或单独线程中调用 |
| **回退策略** | LLM 返回空字符串时应有硬编码回退逻辑 |
| **频率控制** | 不要每帧调 LLM，用帧计数器控制调用频率（建议 ≥2秒一次） |
| **Prompt 设计** | 强制要求输出 JSON 格式，便于解析。包含当前传感器数值作为上下文 |
| **API 密钥** | 笔记本端 `llm_bridge.py` 通过环境变量读取密钥，不要硬编码 |

## 测试方法

在构造函数中已有测试桩，连接成功后会打印日志。要测试完整链路：

```bash
# 笔记本端
python3 tools/llm_bridge.py

# 狗端（或仿真）
ros2 service call /llm_ask cyberdog_race/srv/LLMAsk "{prompt: '前方有障碍物怎么办'}"
```
