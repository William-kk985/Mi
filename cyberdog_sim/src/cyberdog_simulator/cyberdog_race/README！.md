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
  │
  ├─ LCM回调 ───── on_sim_state / on_global_to_robot / on_state_estimator
  │
  ├─ control_loop() (10ms)
  │     ├─ stages_[cur_stage_]->run()
  │     ├─ apply_stage_params()
  │     ├─ 赛段切换逻辑
  │     └─ render_track_frame / render_telemetry_frame (Web)
  │
  ├─ WebStreamer (独立线程, MJPEG HTTP, 8路流)
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
├── scripts/                              # 上机运行脚本（rsync 到 NX 后执行）
│   ├── sync_to_nx.sh                     # ★ 安全同步 VM→NX（防覆盖/删除伙伴改动，见「构建与运行」）
│   ├── sensor_probe.py                   # 传感器探测：逐个 topic 收帧验证（含 TOF/超声/右红外）
│   ├── start_web.sh                      # Web 推流（8路 MJPEG + 环境修复 + 防双开 + 8080自动释放）
│   ├── start_rgb_test.sh                 # RGB imshow 预览 (TEST_BEHAVIOR=10)
│   ├── start_sensor_check.sh             # 逐传感器检查 (TEST_BEHAVIOR=9)
│   ├── start_pitch_test.sh               # 俯仰测试 (TEST_BEHAVIOR=7, 走新接口)
│   ├── start_march_test.sh               # 原地踏步测试 (TEST_BEHAVIOR=11, servo 303 vel=0)
│   ├── pitch_test_servo.py               # ★ 真机低头独立验证脚本（CyberDog2 官方接口）
│   └── record_dataset.py                 # 数据集录制（PNG/MP4 → dataset/）
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
- 深度/D430i 红外 **只做展示，不做检测**，不得在对应的 `on_*` 回调中加入赛段逻辑

**⚠ 稳定运行四大坑（2026-08-06 全部踩过并修复，改 Web 代码前必读）**：

1. **SIGPIPE 必须忽略**（`main.cpp` + `WebStreamer::start()` 已 `signal(SIGPIPE, SIG_IGN)`）：
   浏览器切换流关闭旧连接 → 旧线程 `write()` 触发 SIGPIPE → 默认动作**终止整个进程**
   （不是 SIGSEGV，crash_handler 捕不到）——"一切换就崩"根因
2. **客户端线程必须 detach**，❌ 不能存 `std::vector<std::thread>` + join：
   `pthread_tryjoin_np` 回收底层后 std::thread 仍 joinable，`erase` 析构它 → `std::terminate`（SIGABRT）。
   现方案：spawn 后立即 `detach()`，用 `active_clients_` 原子计数，`stop()` 轮询计数归零
3. **断连检测**：`client_handler` 推流循环用 `poll()` 检测 socket 断开（POLLHUP/POLLERR/recv=0 即退出），
   `frame_cv_.wait_for` 用 1s 短超时轮询。否则浏览器断开后线程永不退出 → `active_clients_` 占满上限
   → 后续请求（含首页）全 **503** → **切换几次后黑屏**
4. **剥离 query string**：路由匹配前 `path = path.substr(0, path.find('?'))`。
   浏览器切换流带 `?t=时间戳`（防缓存），不剥离则 `/stream/debug?t=xxx` 匹配失败 → **404 黑屏**

- 连接断开后 `send_all()` 返回 false → 线程正常退出，不崩溃
- 帧缓冲全部在 `frame_mutex_` 保护下（push 用 swap、读用 copy）；socket 收发超时 1s

**诊断命令**（NX 上）：
```bash
# 200=正常  404=query 问题  503=断连线程堆积
curl -s -o /dev/null -w '%{http_code}\n' 'http://127.0.0.1:8080/stream/debug?t=1'
# 线程数（正常 ~6，>10 说明 client_handler 堆积）
ls /proc/$(pgrep -f race_controlle[r] | head -1)/task | wc -l
```

**Web 遥测（2026-08-07 新增）**：
- 顶栏实时显示：stage / yaw / odom / 身高 / **TOF** / **超声**（前端每 1s 拉 `/api/telemetry`）
- 数据源：`control_loop()` 100Hz 调 `web_streamer_.update_telemetry(...)`
- 新增右红外流 `/stream/infra2`（D430i 右目, stype=10）；下拉框有「D430i左红外/右红外」

**⚠ 真机必须用 `start_web.sh` 启动（RMW 大坑, 2026-08-07 踩过）**：
- 系统 ROS2 用 `rmw_cyclonedds_cpp` + `ROS_DOMAIN_ID=42` + `/etc/mi/cyclonedds.xml`
- 若直接跑二进制 / 没 source `/etc/mi/ros2_env.conf` → 回退默认 FastDDS → **发现不了 sensor_manager**：
  - `ros2 node list` 看不到 race_controller
  - 所有传感器订阅（TOF/超声/图像）收不到 → Web 显示初始值/黑屏
- 自检：`ros2 node list | grep -i race` 有输出 = 正常

**⚠ 测试宏会冻结 Web 遥测（2026-08-07 定位）**：
- `DEBUG_TEST_BEHAVIOR` 定义时，`control_loop()` 首次执行 `timer_->cancel()` + return
- → 遥测只更新一次（启动快照），之后永远不变（超声卡 0、TOF 卡 0.66）
- 症状：`/api/telemetry` 多次采样数值**完全一样**
- 发布/联调前确保 `debug_config.hpp` 中 `DEBUG_TEST_BEHAVIOR`/`TEST_BEHAVIOR` 已注释

### 7. 运动控制

- **真机（CyberDog 2）必须走 ROS2 接口**，❌ 不要用 LCM `robot_control_cmd` mode=21 等铁蛋一代接口（见下方专章）
- 真机低头/姿态：发布 `motion_servo_cmd`（`protocol/msg/MotionServoCmd`），`set_body_pitch()` 已封装
- 发布器需在 `RaceController` 构造中调用 `motion_.attach_motion_servo_pub(this)` 挂载（仅 `REAL_DOG`）
- `motion_servo_cmd` 需 **~20Hz 持续发布**（停发 4 帧 = Servo data lost，运动中止）
- ⚠ `motion_ctrl.cpp` 必须先包含 `debug_config.hpp` 再包含 `motion_ctrl.hpp`（`#ifdef REAL_DOG` 依赖宏顺序）
- 仿真：LCM `robot_control_cmd` 的 `life_count` 必须每帧递增（`++lcm_life_`）
- ⚠ **真机步高（2026-08-08 定位，重要）**：
  - ❌ `motion_servo_cmd.step_height` 字段真机**忽略**（303 不读它，5 种编码上机验证全无差别）
  - ✅ 正确路径：**LCM `robot_control_cmd`(7671) 的 `step_height`**（`set_step_height()` 已封装，直接传米即可）
  - ⚠ 编码是**打包毫米**：`step_height[n] = 高3位毫米 + 低3位毫米`（控制器解码 `(int)%1000*1e-3`）
    - `0.15m → 150mm → 打包 150*1000+150 = 150150`；直接发米 `0.15` 会被 `(int)` 截成 0（旧代码的坑！）
  - ✅ 用法：**起步前设一次**（走/踏步前调 `set_step_height(0.15, 0.15)`），中途不用改
  - ⚠ 0.20m 偏高（真机默认抬腿已 ~0.30m，加 0.20 接近上限、狗不稳），**比赛建议 0.10~0.15**
  - 步高范围：0 ~ 0.35m（打包毫米上限 350）
- `LocoMode` 枚举替代魔法数字

### 8. 新增 .cpp 文件

必须在 `CMakeLists.txt` 的 `set(SOURCES ...)` 中添加，否则链接错误。

### 9. 测试文件隔离

- 测试代码统一放在 `src/test/` 下，`inc/` 放头文件、`src/` 放源文件，与正式代码完全分离
- 每个平台（`real/`、`virtual/`）各自维护 `inc/` + `src/`，自包含
- 测试通过 cmake 条件编译控制，不污染正式版：

```bash
# 真机赛段用测试版
colcon build --cmake-args -DUSE_TEST_REAL_ALL=ON
colcon build --cmake-args -DUSE_TEST_REAL_STAGE3=ON

# 虚拟赛段用测试版
colcon build --cmake-args -DUSE_TEST_STAGE1=ON
colcon build --cmake-args -DUSE_TEST_ALL=ON
```

- 测试类名加 `Test` 后缀（如 `Stage1RealTest`），与正式实现完全独立

---

## 🧪 行为测试（TEST_BEHAVIOR 1~16）

> 独立测试单个动作/算法，定义后替代正常状态机（`control_loop` 只跑测试）。
> 入口：`src/test/function/src/behavior_test.cpp` 的 `run_test()`。

### 启用方式（`debug_config.hpp`）

```cpp
#define DEBUG_TEST_BEHAVIOR      // 开启测试模式
#define TEST_BEHAVIOR 11         // 选择测试编号
```

⚠ **测试完必须注释回**（`// #define DEBUG_TEST_BEHAVIOR`），否则：
- 遥测冻结在启动快照（TOF 卡 0.66、超声卡 0）
- 正式赛段状态机不跑

⚠ **2026-08-08 修复**：测试在**独立线程**运行（`std::thread`），主线程 `spin` 继续派发
ROS2 订阅回调 → 测试期间 TOF/超声/LiDAR 实时可读（以前同步跑会卡死单线程执行器，传感器全冻）。

### 测试清单

| # | 名称 | 函数 | 动作 | 状态 |
|---|---|---|---|---|
| 1 | 跳跃 | `jump_test` | 旧 LCM JUMP_3D ×3 | 仿真 |
| 2 | 扫描找球 | `scan_ball_test` | 左右转找球（旧 set_velocity） | 仿真 |
| 3 | 蹲下 | `crouch_test` | 占位（看 apply_stage_params） | — |
| 4 | 路径点导航 | `navigate_test` | 走到 (2,3)（旧 set_velocity） | 仿真 |
| 5 | 原地转向 | `turn_test` | 旧 set_velocity yaw 2s | 仿真 |
| 6 | 起立趴下 | `stand_lie_test` | stand/locomotion/lie_down | 真机 ✅ |
| 7 | 俯仰角 | `pitch_test` | 低头-0.26→回正→抬头+0.26（201） | 真机 ✅ |
| 8 | 步高(旧) | `step_height_test` | 旧 `set_step_height`（米，真机无效） | ⚠ 弃用 |
| 9 | 传感器检查 | `sensor_check_test` | 5s 后快照打印各传感器 | 真机 ✅ |
| 10 | RGB预览 | `rgb_view_test` | cv::imshow 弹窗（DEBUG_VISION） | — |
| 11 | 原地踏步 | `march_in_place_test` | 303 vel=0 踏步5s | 真机 ✅ |
| 12 | 前进 | `forward_test` | odom闭环前进0.5m @0.3m/s | 真机 ✅ |
| 13 | 前跳30cm | `jump30_test` | ServoCmd 133（30帧@50ms） | 真机 ✅ |
| 14 | 相对转向90° | `turn_angle_test` | abs_yaw闭环，+0.6=左转 | 真机 ✅ 误差~2.4° |
| 15 | 绝对转向90° | `abs_turn_test` | 地图绝对朝向闭环，双向选最近 | 真机 ✅ 误差~2.5° |
| 16 | 步高标定 | `step_height_walk_test` | LCM set_step_height 打包毫米，0.10/0.15/0.20 | 真机 ✅ 0.15基准 |

### 真机验证要点（2026-08-07~08）

- **转向反馈用 `abs_yaw`（global_to_robot.rpy[2]）**，⚠ 真机 IMU yaw 恒 0（realsense 融合死），别用 `sensor_.yaw`
- **跳跃走 ServoCmd（133/132）**，❌ MotionResultCmd 会报 `Command 133 not valid`
- **站立/趴下走 MotionResultCmd（111/101）**，❌ 旧 gamepad LCM 真机无效
- **步高（#16）**：真机只认 LCM `robot_control_cmd`(7671) + **打包毫米**编码（`0.15→150150`），
  ⚠ 303 的 `motion_servo_cmd.step_height` 真机忽略（详见「步高控制」节）；0.15 为稳定基准，0.25 会不稳
- **4腿TOF测抬升**：`sensor.tof_elev_max` = 抬腿峰值（TOF 在四条腿上测 elevation，protocol 注释确认），
  用于量化步高/步态；`sensor.tof_clearance` = 脚着地基线

### 常见坑

- 测试期间传感器实时性靠"独立线程"修复（见上），若 TOF 又卡 0.66 先查测试是否同步跑
- `debug_config.hpp` 里 TEST_BEHAVIOR 的注释只列到 11，实际已有 1~16（以 `run_test()` 的 switch 为准）

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

| 传感器 | Topic (仿真) | Topic (真机, 前缀 `ROBOT_NS`) | 回调 | 数据写入 |
|---|---|---|---|---|
| RGB 相机 | `/RGB_camera/image_raw` | `/image`（camera_server 推流） | `on_rgb` | 视觉检测结果 |
| D430i 红外(左目) | `/D435/infra1/image_raw` | `/camera/infra1/image_rect_raw` | `on_d435_infra1` | Web 展示 |
| D430i 红外(右目) | — | `/camera/infra2/image_rect_raw` | `on_d435_infra2` | Web 展示(/stream/infra2) |
| D430i 深度 | `/D435/depth/image_raw` | `/camera/depth/image_rect_raw` | `on_d435_depth` | Web 伪彩色展示 |
| TOF 头部×2 | — | `/head_tof_payload`（protocol 8x8高程） | `on_tof_head` | tof_clearance（4路最低点, 0.15-0.66m） |
| TOF 尾部×2 | — | `/rear_tof_payload`（protocol 8x8高程） | `on_tof_rear` | tof_clearance |
| 超声波 | — | `/ultrasonic_payload`（Range, 0.1-1.0m） | `on_ultrasonic` | ultrasonic_range（有遮挡才发布） |
| IMU | `/imu` | `/camera/imu`（D430i 内置） | `on_imu` | yaw/pitch/roll |
| LiDAR | `/scan` | `/scan` | `on_lidar` | lidar_front |
| BMS | — | `/bms_status` | `on_bms` | 低电量保护 |
| 触摸 | — | `/touch_status` | `on_touch`(TODO) | 紧急停止 |
| 里程计 | LCM `simulator_state` | LCM `global_to_robot` | `on_sim_state`/`on_global_to_robot` | odom_x/y, body_height |
| 状态估计 | — | LCM `state_estimator` | `on_state_estimator` | body_height 覆盖 |

> 真机 topic 值以 `debug_config.hpp` 为准（2026-07-31 上机确认）。

---

## 🐕 真机运动控制 — CyberDog 2 官方接口（2026-08-06 验证）

> ⚠️ **不要用 LCM `robot_control_cmd` 的 `mode=21 (POSE_CTRL)` 控制真机低头**——
> 那是铁蛋一代接口。CyberDog 2 的 NX `motion_manager` 会把外部 LCM mode=21
> 错误映射成 motion 303 = **WALK_USERTROT（走路步态）**，且与 motion_action 自己发的
> LCM 在 7671 上互相覆盖（日志现象：`mode=21 ... then mode=7`）。正确接口是 ROS2 topic：

### 接口

| 项 | 值 |
|---|---|
| Topic | `ROBOT_NS "/motion_servo_cmd"`（`protocol/msg/MotionServoCmd`） |
| 姿态控制 | `motion_id = 201`（FORCECONTROL_DEFINITIVELY），`rpy_des=[roll, pitch, yaw]` |
| 低头 | `pitch` 负值（限 **-0.25** rad），抬头正值（限 +0.30） |
| 机身高度 | `pos_des=[0, 0, 0.235]`（官方默认） |
| 优先级 | `cmd_source = -1`（DEBUG 最高优先级） |
| 频率 | **~20Hz 持续发布**（停发 4 帧 = Servo data lost，运动退出） |

### motion_id 速查

| motion_id | 名称 | 用途 |
|---|---|---|
| 0 | ESTOP | 急停 |
| 101 | GETDOWN | 趴下 |
| 111 | RECOVERYSTAND | 恢复站立（发 servo 201 时会**自动先站起**） |
| 112 | WALK_STAND | 站立 |
| 201 | FORCECONTROL_DEFINITIVELY | 姿态控制（低头/抬头/roll/yaw）★ |
| 211 | POSECONTROL_DEFINITIVELY | 位控姿态（备选） |
| 303 | WALK_USERTROT | 自定义小跑（⚠ 旧 mode=21 会被错误映射到这里） |

### 代码用法

```cpp
// C++（已封装，REAL_DOG 下自动生效）
motion_.set_body_pitch(-0.2f);   // 低头 0.2 rad（负=低头）
// ⚠ 需在循环中 ~20Hz 持续调用保持，停发即退出

// 步高（2026-08-08 真机验证）：起步前设一次，走 LCM robot_control_cmd(7671)
motion_.set_step_height(0.15f, 0.15f);   // 内部转打包毫米 150150，真机生效

// 独立验证脚本（不依赖 C++ 工程，直接在 NX 上跑）
python3 scripts/pitch_test_servo.py --stand --pitch -0.2 --hold 3
```

### 步高控制（2026-08-08 真机验证，别走错通道）

| 通道 | 是否生效 | 说明 |
|---|---|---|
| `motion_servo_cmd.step_height`（303） | ❌ 忽略 | 5 种编码（米/毫米/打包）上机验证全无差别，固件不读 |
| LCM `robot_control_cmd`(7671) `step_height` | ✅ 生效 | `set_step_height()` 已封装；编码=打包毫米（`0.15→150150`） |
| 旧 gamepad（7667） | ❌ 无效 | 铁蛋一代接口 |

- 调用时机：**步态起步前设一次**即可，中途改无效
- 幅度：0.20m 会接近真机上限（默认抬腿已 ~0.30m，狗不稳），建议 0.10~0.15

### 诊断

- motion_manager 日志：`/home/mi/.ros/log/motion_manager_*.log`
  - `ServoCmd: mode, gait_id, life_count, duration` = 发给 MR813 的 LCM 命令
  - `Receive ServoCmd from -1 with motion_id: 201` = 收到外部 servo 命令
  - `Servo data lost time with 4 times` = 发布中断（需持续发布）
  - `Command 111 not valid` = 处于 servo 状态时直接发 ResultCmd 被拒（属正常，发 servo 会自动先站）
- `motion_managermachine_service` 是**节点生命周期**状态机（Uninitialized=0 / Active=4），
  **不是运动状态**；`code=8` = 状态名不支持，勿用它查询运动状态
- 全局状态查询：`ros2 service call .../machine_state_valget std_srvs/srv/Trigger`

---

## 构建与运行

```bash
# 仿真（默认，VM）
cd /home/kaka/Mi/cyberdog_sim
colcon build --merge-install --packages-select cyberdog_race

# 真机（NX 上构建）
# 1. VM 编辑后同步到 NX：⚠ 必须用安全脚本，❌ 勿用 rsync --delete
#    （--delete 会删除/覆盖 NX 上伙伴改了但没 git 同步的文件——历史教训 .github/ 被删）
bash /home/kaka/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race/scripts/sync_to_nx.sh
# 2. NX 上构建（真机需 protocol 包——NX 有、仿真无 → CMake 已 QUIET 保护）：
ssh cyberdog "source /etc/mi/ros2_env.conf && cd /SDCARD/race_ws && \
  colcon build --merge-install --packages-select cyberdog_race"
# 3. 运行：⚠ 必须用 start_web.sh（内部 source /etc/mi/ros2_env.conf 设置 CycloneDDS）
#    直接跑二进制会因 RMW 不一致导致传感器订阅全断（见「Web 推流」RMW 大坑）
ssh cyberdog "cd /SDCARD/race_ws/src/cyberdog_race/scripts && ./start_web.sh"

# 笔记本 LLM 桥接（PROXY 模式）
source install/setup.bash
export LLM_API_KEY="sk-your-key-here"
python3 tools/llm_bridge.py
```


---

## 完整文档

参见 [`开发状态！.md`](/开发状态！.md) 获取所有编译宏说明、API 路由表、上机注意事项。
