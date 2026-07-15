# cyberdog_race — CyberDog 2 比赛控制器

> **架构规范文档** — 修改前必读，避免破坏代码设计约定。

---

## 整体架构

```
race_controller (ROS2 Node, 100Hz timer)
  │
  ├─ on_rgb()     ─── LaneDetector / BallDetector / Stage4Detector
  │                     ↓
  │                  sensor_ (SensorData, 互斥锁保护)
  │
  ├─ on_imu()     ─── sensor_.yaw/pitch/roll
  ├─ on_lidar()   ─── sensor_.lidar_front
  ├─ on_d435_*()  ─── WebStreamer push (只做展示，不做检测)
  ├─ on_fish_*()  ─── WebStreamer push (只做展示，不做检测)
  │
  ├─ LCM回调 ───── on_sim_state / on_global_to_robot / on_state_estimator
  │
  ├─ control_loop() (10ms)
  │     ├─ stages_[cur_stage_]->run()
  │     ├─ apply_stage_params()
  │     ├─ 赛段切换逻辑
  │     └─ render_track_frame / render_telemetry_frame (Web)
  │
  ├─ WebStreamer (独立线程, MJPEG HTTP, 9路流)
  └─ LLMHelper (PROXY/API 双模式)
```

### 数据流三大原则

1. **传感器回调只写锁、不做检测**：`on_rgb` 把检测放到锁外（~5-30ms），锁内仅写共享数据（<0.01ms）
2. **赛段只读传感器数据**：`StageBase::run()` 读取 `sensor_` 时自行加锁快照
3. **Web 推流是纯展示**：渲染方法在 `control_loop` 中调用，锁内快照后锁外渲染

---

## 文件结构规范

```
cyberdog_race/
├── CMakeLists.txt                        # 编译配置（新增 .cpp 必须加 SOURCES）
├── package.xml                           # ROS2 包清单
├── README！.md                           # ← 本文档
│
├── doc/                                  # 文档
│   └── LLM_GUIDE.md                      # LLM 集成指南
│
├── srv/                                  # ROS2 服务定义
│   └── LLMAsk.srv
│
├── include/cyberdog_race/
│   ├── debug_config.hpp                  # ★ 所有编译时宏定义（真机/仿真切换）
│   ├── race_controller.hpp               # 主控制器类声明
│   ├── motion_ctrl.hpp                   # LCM 运动控制接口
│   ├── sensor_data.hpp                   # 传感器共享数据 POD
│   ├── llm_helper.hpp                    # LLM 通信辅助
│   │
│   ├── stages/
│   │   ├── stage_base.hpp                # 赛段基类（纯虚接口）
│   │   ├── virtual/                      # 仿真赛段实现
│   │   │   ├── stage1.hpp ~ stage6.hpp
│   │   └── real/                         # 真机赛段骨架（独立实现，互不干扰）
│   │       ├── stage1_real.hpp ~ stage6_real.hpp
│   │       └── README.md
│   │
│   ├── utils/
│   │   ├── web_streamer.hpp              # 嵌入式 MJPEG HTTP 推流
│   │   └── led_indicator.hpp             # LED 赛段指示（框架，等 bridges 包）
│   │
│   └── vision/
│       ├── virtual/                      # 仿真视觉检测器
│       │   ├── lane_detector.hpp
│       │   ├── ball_detector.hpp
│       │   └── stage4_detector.hpp
│       └── real/                         # 真机视觉骨架
│           ├── lidar_locator.hpp
│           └── yolo_detector.hpp
│
└── src/
    ├── main.cpp                          # 入口 + spin + 关机序列
    ├── race_controller.cpp               # 主控制器实现
    ├── motion_ctrl.cpp                   # LCM 运动指令
    ├── llm_helper.cpp                    # LLM 通信
    ├── stages/virtual/                   # 仿真赛段 .cpp
    ├── stages/real/                      # 真机赛段 .cpp（骨架）
    ├── utils/web_streamer.cpp            # HTTP 服务
    ├── vision/virtual/                   # 视觉检测器 .cpp
    ├── test/                             # 测试代码（cmake 条件编译）
    │   ├── function/                     # 通用测试函数
    │   │   ├── inc/behavior_test.hpp     # 行为测试 + 通用链路测试
    │   │   └── src/behavior_test.cpp     # 行为测试 + 通用链路测试实现
    │   ├── real/                         # 真机赛段测试
    │   │   ├── inc/                      # 真机赛段测试头文件
    │   │   │   ├── stage1_real_test.hpp ~ stage6_real_test.hpp
    │   │   └── src/                      # 真机赛段测试源文件
    │   │       ├── stage1_real_test.cpp ~ stage6_real_test.cpp
    │   └── virtual/                      # 虚拟赛段测试
    │       ├── inc/
    │       └── src/
```

---

## 🚫 关键约定（AI 修改时不得违反）

### 1. 编译宏体系 — `debug_config.hpp`

- **`REAL_DOG`** 定义时编译真机代码，注释时编译仿真代码
- **`LLM_MODE_PROXY` / `LLM_MODE_API`** 二选一，不能同时定义（有 `#error` 保护）
- **`ENABLE_WEB_STREAMING`** 注释掉整个 Web 推流零开销
- 所有调试宏（`DEBUG_VISION` 等）正式比赛前必须注释掉

⚠️ **不要**在 `.cpp` 或 `.hpp` 中硬编码 topic 名称，统一走宏。

### 2. 线程安全

| 数据 | 写者 | 读者 | 保护方式 |
|---|---|---|---|
| `sensor_` 全部字段 | 所有 `on_*` 回调 | `control_loop`, `render_*`, StageBase::run | `sensor_mutex_` 互斥锁 |
| `SensorData::vision_result` | `on_rgb` | `Stage4::run` | 低频无锁（数据竞争容忍） |
| `WebStreamer` 缓冲 | `on_rgb` / `render_*` | HTTP 客户端线程 | `frame_mutex_` + `condition_variable` |
| `odom_history_` | `control_loop` | `render_track_frame` | 读时锁内快照 |

**黄金法则**：锁内操作不得超过 0.1ms。OpenCV 检测、JPEG 编码等耗时操作必须放在锁外。

### 3. 颜色空间 — BGR 统一

所有检测器、调试画面、Web 推流统一使用 **BGR** 颜色空间：

```
on_rgb()
  → cv_bridge::toCvShare(msg, "bgr8")   ← cv_bridge 自动处理源编码 (rgb8→bgr8)
  → LaneDetector::detect(cv_img, BGR)    ← 内部做 BGR→HSV 转换
  → BallDetector::detect(cv_img, BGR)    ← 同上
  → Stage4Detector::detect(cv_img, BGR)  ← 同上
```

🚫 **不要**在 `on_rgb` 之外单独订阅 ROS2 图像再转 BGR——统一入口已处理编码差异。

### 4. 仿真 / 真机隔离

| 层 | 仿真 | 真机 |
|---|---|---|
| 视觉检测 | `stages/virtual/` + `vision/virtual/` | 可复用虚拟检测器（BGR 统一后） |
| 赛段逻辑 | `Stage1~6` | `Stage1Real~6Real`（独立类，互不干扰） |
| 里程计 | LCM `simulator_state` | LCM `global_to_robot`（`localization_lcmt`） |
| 相机 topic | `/RGB_camera/image_raw` | `/image_rgb`（lifecycle 激活） |
| LiDAR | `/scan` (LaserScan) | `/scan`（⚠ 可能为自定义 ScanMsg） |

真机赛段（`*_real.hpp`）完全独立于仿真赛段的类继承体系，**不允许互相引用**。

### 5. LLM 模式

```
// #define LLM_MODE_PROXY   ← 比赛推荐（狗→ROS2 Srv→笔记本→云端）
// #define LLM_MODE_API     ← 备选（狗→libcurl→云端直连）
// 都不定义 → 零开销，不链接 LLM
```


PROXY 模式需要笔记本运行 `tools/llm_bridge.py`。
API 模式需要 `libcurl`，`CMakeLists.txt` 已做 `find_package(CURL QUIET)`。

### 6. Web 推流

- 所有相机展示流定义在 `ENABLE_WEB_STREAMING` 块中
- 新增流需要：`web_streamer.hpp` 加 push 方法 + 缓冲 → `web_streamer.cpp` 加路由 + stype + switch 分支
- 鱼眼/深度/D430i 红外 **只做展示，不做检测**，不得在对应的 `on_*` 回调中加入赛段逻辑

### 7. 运动控制

- LCM `robot_control_cmd` 的 `life_count` 必须每帧递增（`++lcm_life_`）
- 步高 clamp 在 [0, 0.35]m
- `LocoMode` 枚举替代魔法数字

### 8. 新增 .cpp 文件

必须在 `CMakeLists.txt` 的 `set(SOURCES ...)` 中添加，否则链接错误。

### 9. 测试文件隔离

- 测试代码统一放在 `src/test/` 下，`inc/` 放头文件、`src/` 放源文件，与正式代码完全分离
- 每个平台（`real/`、`virtual/`）各自维护 `inc/` + `src/`，自包含
- 测试通过 cmake 条件编译控制，不污染正式版：

```bash
# 全部真机赛段用测试版
colcon build --cmake-args -DUSE_TEST_REAL_ALL=ON
真机赛段用测试版
colcon build --cmake-args -DUSE_TEST_REAL_STAGE3=ON

# 虚拟赛段用测试版
colcon build --cmake-args -DUSE_TEST_STAGE1=ON
colcon build --cmake-args -DUSE_TEST_ALL
colcon build --cmake-args -DUSE_TEST_REAL_STAGE3=ON
```

- 测试类名加 `Test` 后缀（如 `Stage1RealTest`），与正式实现完全独立

---

## 如何添加一个新赛段

1. `stages/virtual/stageN.hpp` 继承 `StageBase`，实现 `init/run/is_done`
2. `stages/virtual/stageN.cpp` 实现
3. `race_controller.hpp` 头文件 include
4. `race_controller.cpp` 构造函数中 `stages_[N-1] = std::make_unique<StageN>(...)`
5. 真机版：`stages/real/stageN_real.hpp` 继承 `StageBase`，独立实现
6. `CMakeLists.txt` SOURCES 添加 `.cpp`

---

## 传感器一览

| 传感器 | Topic (仿真) | Topic (真机) | 回调 | 数据写入 |
|---|---|---|---|---|
| RGB 相机 | `/RGB_camera/image_raw` | `/image_rgb` | `on_rgb` | 视觉检测结果 |
| 左鱼眼 | 复用 RGB | `/image_left` | `on_fish_eye_left` | Web 展示 |
| 右鱼眼 | 复用 RGB | `/image_right` | `on_fish_eye_right` | Web 展示 |
| D430i 红外 | `/D435/infra1/image_raw` | `/camera/infra1/image_rect_raw` | `on_d435_infra1` | Web 展示 |
| D430i 深度 | `/D435/depth/image_raw` | `/camera/depth/image_rect_raw` | `on_d435_depth` | Web 伪彩色展示 |
| IMU | `/imu` | `/imu` | `on_imu` | yaw/pitch/roll |
| LiDAR | `/scan` | `/scan` (⚠ ScanMsg) | `on_lidar` | lidar_front |
| BMS | — | `bms_status` | `on_bms` | 低电量保护 |
| 里程计 | LCM `simulator_state` | LCM `global_to_robot` | `on_sim_state`/`on_global_to_robot` | odom_x/y, body_height |
| 状态估计 | — | LCM `state_estimator` | `on_state_estimator` | body_height 覆盖 |

```bash
# 仿真（默认）
cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash
colcon build --merge-install --packages-select cyberdog_race

# 真机：取消 #define REAL_DOG 的注释后同上

# 笔记本 LLM 桥接
source install/setup.bash
export LLM_API_KEY="sk-your-key-here"
python3 tools/llm_bridge.py
```
# 笔记本 LLM 桥接
source install/setup.bash
export LLM_API_KEY="sk-your-key-here"
python3 tools/llm_bridge.py


---

## 完整文档

参见 [`开发状态！.md`](/开发状态！.md) 获取所有编译宏说明、API 路由表、上机注意事项。
