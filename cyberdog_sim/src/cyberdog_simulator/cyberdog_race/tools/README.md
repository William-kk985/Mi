# tools/ — 开发工具脚本索引

> 按功能分类整理（2026-09-02）。所有脚本在真机/VM 上独立使用，路径引用已更新。

## 目录结构

| 子目录 | 用途 | 关键脚本 |
|---|---|---|
| `build/` | 构建/部署/仿真 | `build_race.sh`（真机编译 VM→NX）、`start_sim.sh`（仿真一键启动） |
| `llm/` | LLM 桥接 | `llm_bridge.py`（PROXY 模式：狗→笔记本→云端） |
| `vision/` | 视觉/HSV/图像分析 | `analyze_hsv.py`、`pick_hsv.py`、`select_hsv.py`、`calib_ball.py`、`verify_orange.py`、`test_models.py` |
| `probe/` | 传感器/里程计/IMU 探测 | `probe_odom.py`、`probe_myimu.py`、`probe_yaw3.py`、`probe_yaw_cmp.py`、`hw_profile.sh`、`s3_diag.sh`、`qos_check.sh` |
| `camera/` | 相机诊断/测试 | `diag_*.sh`、`test_gc02*.sh`、`test_ov13*.sh`、`test_v4l2.sh`、`verify_gc02.sh`、`shot_orange.py`、`shot_rgb.py` |
| `record/` | 录像/回放/数据处理 | `rec_video.py`、`rec_video_multi.py`、`replay_video.py`、`process_video.py`、`video_to_frames.py`、`save_infra.sh` |
| `rviz/` | rviz/建图/TCP 转发 | `start_rviz.sh`、`start_mapping.sh`、`scan_forward(_v2).py`、`scan_receive(_v2).py`、`rviz_cyberdog.rviz` |
| `scripts/` | sh 便捷/部署脚本 | `start_race.sh`、`start_web.sh`、`sync_to_nx.sh` 等（源自 cyberdog_race/scripts/） |
| `robot/` | 真机控制 | `stop_all.sh` |

## 常用命令速查

```bash
# 仿真一键启动（容器 cyberdog）
bash ~/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race/tools/build/start_sim.sh

# 真机编译（VM → 安全同步 → NX 编译）
bash ~/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race/tools/build/build_race.sh

# LLM 桥接（比赛 PROXY 模式，笔记本跑）
export LLM_API_KEY="sk-..."
python3 ~/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race/tools/llm/llm_bridge.py

# 跨机器 rviz 可视化（VM 看 NX 雷达/地图/点云）
bash ~/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race/tools/rviz/start_rviz.sh

# 关真机所有节点
bash ~/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race/tools/robot/stop_all.sh

# 安全同步源码到 NX（含部署 sh → NX cyberdog_race/scripts/）
bash ~/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race/tools/scripts/sync_to_nx.sh
```

## 退役说明（2026-09-02）

- `build_bins.sh` / `save_bin.sh`：**已删除**。原"sed 改宏 + 17 个二进制"的多版本机制已被 `launch` 参数选路线（`stage4_impl`）取代，见包内 `launch/race.launch.py`。
- `tools/__pycache__/`：已删除（Python 缓存）。

## 引用关系提醒

- `rviz/start_rviz.sh` 用绝对路径引用 `rviz/scan_receive_v2.py` 和 `rviz/rviz_cyberdog.rviz`——**这两个文件不要移出 rviz/ 目录**，否则要同步改路径。
- 各子目录内脚本基本相互独立，可单独使用。
