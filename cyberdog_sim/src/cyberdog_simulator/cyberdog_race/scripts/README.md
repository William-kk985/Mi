# scripts/ — ROS py 层

> 2026-09-02 整理：这里只放 **ROS 相关 Python**（节点的 py 原型/测试）。
> sh 便捷/部署脚本在包内 `tools/scripts/`（与 scripts/ 同级）。

## 定位

- 主代码（C++）做简单测试时，可先在这里用 **py 写逻辑**（如未来要尝试的 **LLM 决策**），跑通后再转成 C++（`src/`）。
- 与 `tools/` 的区分：
  - **`cyberdog_race/scripts/`（本目录）= ROS py 原型/测试层**——能跑 ROS 逻辑的 py 版本
  - **`tools/` = 通用开发工具层**——py 工具 + sh 便捷脚本

## 现有脚本

| 脚本 | 用途 |
|---|---|
| `s4_detect_server.py` | Stage4 可乐/足球 ONNX 推理服务（unix socket `/tmp/s4_detect.sock`，被 `start_race.sh` 拉起） |
| `sensor_probe.py` | 传感器探测：逐个 topic 收帧验证（含 TOF/超声/右红外） |
| `pitch_test_servo.py` | 真机低头独立验证脚本（CyberDog2 官方接口 201） |
| `record_dataset.py` | 数据集录制（PNG/MP4 → dataset/） |

## 约定

1. 新增 ROS py 原型放这里（如未来的 `llm_decision.py`），验证后转 C++。
2. 部署入口 sh（`start_race.sh` / `start_web.sh` 等）在 `~/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race/tools/scripts/`，由 `sync_to_nx.sh` 同步到 NX 的 `cyberdog_race/scripts/`（**NX 路径不变**，部署命令照旧）。
