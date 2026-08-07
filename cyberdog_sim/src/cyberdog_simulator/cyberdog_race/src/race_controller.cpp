#include "cyberdog_race/race_controller.hpp"

// ============================================================
// 调试模式说明（修改 debug_config.hpp 后重新 build）：
//   DEBUG_SINGLE_STAGE N  → 只跑第N赛段，赛段结束后停止，不切换
//   DEBUG_START_STAGE  N  → 从第N赛段开始，走完整状态机
//   两者都不定义          → 正式比赛模式，从第1赛段完整跑
//   DEBUG_VISION  → 视觉 imshow 可视化
//   DEBUG_MOTION  → 运动指令日志
//   DEBUG_SENSOR  → 传感器数据日志
//   DEBUG_STAGE   → 状态机切换日志
// ============================================================

RaceController::RaceController() : Node("race_controller") {
#if defined(DEBUG_SINGLE_STAGE)
    cur_stage_ = DEBUG_SINGLE_STAGE - 1;
    single_stage_mode_ = true;
    RCLCPP_WARN(get_logger(), "[DEBUG] Single stage mode: only running stage %d", DEBUG_SINGLE_STAGE);
#elif defined(DEBUG_START_STAGE)
    cur_stage_ = DEBUG_START_STAGE - 1;
    RCLCPP_WARN(get_logger(), "[DEBUG] Starting from stage %d", DEBUG_START_STAGE);
#else
    cur_stage_ = 0;
#endif

    auto qos_be = rclcpp::QoS(10).best_effort();
    sub_rgb_ = create_subscription<sensor_msgs::msg::Image>(
        TOPIC_RGB_CAMERA, qos_be,
        [this](sensor_msgs::msg::Image::SharedPtr msg) { on_rgb(msg); });
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        TOPIC_IMU, qos_be,
        [this](sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });
    sub_lidar_ = create_subscription<sensor_msgs::msg::LaserScan>(
        TOPIC_LIDAR, 10,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { on_lidar(msg); });
    sub_d435_infra1_ = create_subscription<sensor_msgs::msg::Image>(
        TOPIC_D435_INFRA1, qos_be,
        [this](sensor_msgs::msg::Image::SharedPtr msg) { on_d435_infra1(msg); });
    sub_d435_infra2_ = create_subscription<sensor_msgs::msg::Image>(
        TOPIC_D435_INFRA2, qos_be,
        [this](sensor_msgs::msg::Image::SharedPtr msg) { on_d435_infra2(msg); });
    sub_d435_depth_ = create_subscription<sensor_msgs::msg::Image>(
        TOPIC_D435_DEPTH, qos_be,
        [this](sensor_msgs::msg::Image::SharedPtr msg) { on_d435_depth(msg); });
#ifdef REAL_DOG
    sub_head_tof_ = create_subscription<protocol::msg::HeadTofPayload>(
        TOPIC_TOF_HEAD, 10,
        [this](protocol::msg::HeadTofPayload::SharedPtr msg) { on_tof_head(msg); });
    sub_rear_tof_ = create_subscription<protocol::msg::RearTofPayload>(
        TOPIC_TOF_REAR, 10,
        [this](protocol::msg::RearTofPayload::SharedPtr msg) { on_tof_rear(msg); });
    sub_ultrasonic_ = create_subscription<sensor_msgs::msg::Range>(
        TOPIC_ULTRASONIC, 10,
        [this](sensor_msgs::msg::Range::SharedPtr msg) { on_ultrasonic(msg); });
#endif


#ifdef REAL_DOG
    // BMS 电池监控（bms_status → protocol::msg::BmsStatus，暂时用 Float32MultiArray 占位）
    // 上机后若拿到 bridges 包，替换为 protocol::msg::BmsStatus 类型
    sub_bms_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        TOPIC_BMS, 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) { on_bms(msg); });

    // TODO: 触摸紧急停止 — 需要 protocol::msg::TouchStatus（从真狗 bridges 包获取）
    // touch_status topic 格式: Header header + int32 touch_state + uint64 timestamp
    // touch_state: 0x01=单击 0x03=双击 0x07=长按
    // 拿到 bridges 包后取消下面注释:
    // sub_touch_ = create_subscription<protocol::msg::TouchStatus>(
    //     TOPIC_TOUCH, 10,
    //     [this](protocol::msg::TouchStatus::SharedPtr msg) { on_touch(msg); });

    // ═══ TODO: TOF 四腿离地传感器（需 HeadTofPayload/RearTofPayload 类型） ═══
    // 真狗4个TOF: LEFT_HEAD/RIGHT_HEAD/LEFT_REAR/RIGHT_REAR, 有效150-660mm, 8x8高程10Hz
    // 用于 Stage5 独木桥检测: tof_clearance < 0.1m 表示偏离桥面
    // sub_head_tof_ = create_subscription<protocol::msg::HeadTofPayload>(
    //     "head_tof_payload", 10,
    //     [this](protocol::msg::HeadTofPayload::SharedPtr msg) {
    //         float min_h = 0.66f;
    //         for (float v : msg->left_head.data)  if (v < min_h) min_h = v;
    //         for (float v : msg->right_head.data) if (v < min_h) min_h = v;
    //         sensor_.tof_clearance = min_h;
    //     });
    // sub_rear_tof_ = create_subscription<protocol::msg::RearTofPayload>(
    //     "rear_tof_payload", 10,
    //     [this](protocol::msg::RearTofPayload::SharedPtr msg) {
    //         float min_h = sensor_.tof_clearance;
    //         for (float v : msg->left_rear.data)  if (v < min_h) min_h = v;
    //         for (float v : msg->right_rear.data) if (v < min_h) min_h = v;
    //         sensor_.tof_clearance = min_h;
    //     });
#endif

    yaml_pub_ = create_publisher<cyberdog_msg::msg::YamlParam>("yaml_parameter", 10);
    auto param = cyberdog_msg::msg::YamlParam();
    param.name = "use_rc";
    param.kind = 2;
    param.s64_value = 0;
    param.is_user = 0;
    rclcpp::sleep_for(std::chrono::seconds(1));
    yaml_pub_->publish(param);

#ifdef REAL_DOG
    // 挂载 CyberDog2 官方姿态控制发布器（set_body_pitch 走 motion_servo_cmd）
    motion_.attach_motion_servo_pub(this);
    // 挂载 MotionResultCmd 服务客户端（跳跃/站立/趴下官方动作）
    motion_.attach_motion_result_client(this);
#endif

#if defined(DEBUG_TEST_BEHAVIOR) && (TEST_BEHAVIOR == 9 || TEST_BEHAVIOR == 10)
    // 传感器/RGB预览模式：跳过运动控制
    RCLCPP_WARN(get_logger(), "[Test] 跳过运动初始化");
#else
    try {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        motion_.recovery();
        rclcpp::sleep_for(std::chrono::seconds(2));
        motion_.locomotion();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        // ⚠ 启动保持水平，不低头（-0.26 是低头测试遗留，真机启动就低头会出事故）
        motion_.set_pitch(0.0f);
        RCLCPP_INFO(get_logger(), "Motion init OK");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Motion init FAILED: %s", e.what());
    }
#endif

#if defined(DEBUG_TEST_BEHAVIOR) && (TEST_BEHAVIOR == 9 || TEST_BEHAVIOR == 10)
    // test 模式不创建赛段
    RCLCPP_WARN(get_logger(), "[Test] 跳过赛段初始化");
#elif defined(ENABLE_WEB_STREAMING) && !defined(REAL_DOG)  // 仿真Web正常创建赛段
    stages_[0] = std::make_unique<Stage1>(motion_, sensor_);
    stages_[1] = std::make_unique<Stage2>(motion_, sensor_);
    stages_[2] = std::make_unique<Stage3>(motion_, sensor_);
    stages_[3] = std::make_unique<Stage4>(motion_, sensor_);
    stages_[4] = std::make_unique<Stage5>(motion_, sensor_);
    stages_[5] = std::make_unique<Stage6>(motion_, sensor_);

    if (cur_stage_ == 2) lane_detector_.set_mode(LaneMode::RELAXED);
    if (cur_stage_ == 5) ball_detector_.reset_filter();
    stages_[cur_stage_]->init();
#endif

    if (lcm_sub_.good()) {
#ifdef REAL_DOG
        lcm_sub_.subscribe(LCM_ODOM_CHANNEL,
                           &RaceController::on_global_to_robot, this);
        lcm_sub_.subscribe(LCM_STATE_ESTIMATOR,
                           &RaceController::on_state_estimator, this);
        RCLCPP_INFO(get_logger(), "[RealDog] LCM odom: %s, state: %s",
                    LCM_ODOM_CHANNEL, LCM_STATE_ESTIMATOR);
#else
        lcm_sub_.subscribe("simulator_state", &RaceController::on_sim_state, this);
#endif
        lcm_running_ = true;
        lcm_thread_ = std::thread([this]() {
            while (lcm_running_) lcm_sub_.handleTimeout(100);
        });
    } else {
        RCLCPP_WARN(get_logger(), "LCM init failed, odom will be unavailable");
    }

    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() { control_loop(); });

#ifdef ENABLE_WEB_STREAMING
    web_streamer_.start(WEB_STREAM_PORT, 4);
    RCLCPP_INFO(get_logger(), "[WebStreamer] Dual-stream MJPEG on http://0.0.0.0:%d (max 4 clients)", WEB_STREAM_PORT);
#endif

#if defined(LLM_MODE_PROXY)
    if (llm_.init(this)) {
        RCLCPP_INFO(get_logger(), "[LLM] PROXY connected to %s", LLM_SERVICE_NAME);
    } else {
        RCLCPP_WARN(get_logger(), "[LLM] PROXY: %s not available", LLM_SERVICE_NAME);
    }
#elif defined(LLM_MODE_API)
    if (llm_.init()) {
        RCLCPP_INFO(get_logger(), "[LLM] API ready, endpoint: %s", LLM_DEFAULT_URL);
    } else {
        RCLCPP_WARN(get_logger(), "[LLM] API init failed");
    }
#endif

    RCLCPP_INFO(get_logger(), "Race controller started, stage %d", cur_stage_ + 1);
}

RaceController::~RaceController() {
#ifdef ENABLE_WEB_STREAMING
    web_streamer_.stop();
#endif
    lcm_running_ = false;
    if (lcm_thread_.joinable()) lcm_thread_.join();
}

// ── 统一下发赛段参数（仅发送变化的参数，避免每帧轰炸） ──
void RaceController::apply_stage_params() {
    if (!yaml_pub_) return;
    auto* s = stages_[cur_stage_].get();

    // 身高 + roll + pitch（仅变化时发）
    float h = s->get_desired_height();
    if (std::abs(h - last_sent_height_) > 0.01f) {
        last_sent_height_ = h;
        cyberdog_msg::msg::YamlParam p;
        p.name = "des_roll_pitch_height";
        p.kind = 3; p.is_user = 1;
        p.vecxd_value[0] = s->get_desired_roll();
        p.vecxd_value[1] = 0.0;
        p.vecxd_value[2] = h;
        yaml_pub_->publish(p);
    }

    // 步高上限（仅变化时发）
    float sh = s->get_desired_step_height();
    if (std::abs(sh - last_sent_step_h_) > 0.01f) {
        last_sent_step_h_ = sh;
        cyberdog_msg::msg::YamlParam p2;
        p2.name = "step_height_max";
        p2.kind = 1; p2.is_user = 1;
        p2.double_value = sh;
        yaml_pub_->publish(p2);
    }

    // RC 模式（仅在变化时发）
    bool rc = s->needs_rc_mode();
    if (rc != last_rc_mode_) {
        last_rc_mode_ = rc;
        cyberdog_msg::msg::YamlParam prc;
        prc.name = "use_rc";
        prc.kind = 2;
        prc.s64_value = rc ? 1 : 0;
        prc.is_user = 0;
        yaml_pub_->publish(prc);
#ifdef DEBUG_STAGE
        fprintf(stderr, "\033[1;35m[Main] use_rc=%d (Stage%d)\033[0m\n", rc ? 1 : 0, cur_stage_ + 1);
#endif
    }

    // 赛段专属参数（只在切赛段后发一次）
    if (cur_stage_ != extra_params_stage_) {
        extra_params_stage_ = cur_stage_;
        for (auto& ep : s->get_extra_params()) {
            cyberdog_msg::msg::YamlParam pem;
            pem.name = ep.name;
            pem.kind = 1; pem.is_user = 1;
            pem.double_value = ep.value;
            yaml_pub_->publish(pem);
        }
    }

#ifdef REAL_DOG
    // ── 真机备选：LCM control_parameter 通道下发参数 ──
    // TODO: SSH 进真狗 `lcm-spy` 确认 control_parameter 通道是否存在及类型
    // control_parameter_request_lcmt req;
    // snprintf(req.name, sizeof(req.name), "des_roll_pitch_height");
    // snprintf(req.value, sizeof(req.value), "%.3f %.3f %.3f",
    //          stages_[cur_stage_]->get_desired_roll(), 0.0,
    //          stages_[cur_stage_]->get_desired_height());
    // req.parameterKind = 3;
    // lcm_sub_.publish(LCM_CMD_EXEC, &req);
#endif
}

// ── 100Hz 控制循环 ──
void RaceController::control_loop() {
#ifdef ENABLE_WEB_STREAMING
    // 遥测推给 Web（/api/telemetry）：赛道/yaw/odom/身高/TOF/超声
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        web_streamer_.update_telemetry(
            cur_stage_ + 1, sensor_.yaw, sensor_.odom_x, sensor_.odom_y,
            sensor_.body_height, sensor_.tof_clearance, sensor_.ultrasonic_range);
    }
    // ⚠ 轨迹记录+渲染必须放这里（真机 stages_ 为空会在下方提前 return，末尾的渲染永远不执行）
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        odom_history_.emplace_back(sensor_.odom_x, sensor_.odom_y);
    }
    if (odom_history_.size() > 400) odom_history_.pop_front();
    if (++track_render_counter_ >= 20) { track_render_counter_ = 0; render_track_frame(); }
    if (++telem_render_counter_ >= 20) { telem_render_counter_ = 0; render_telemetry_frame(); }
#endif
    // 行为测试模式：替代正常赛段
#ifdef DEBUG_TEST_BEHAVIOR
    static bool test_ran = false;
    if (!test_ran) {
        test_ran = true;
        behavior::run_test(motion_, sensor_, TEST_BEHAVIOR);
        motion_.stop();
        timer_->cancel();
        RCLCPP_WARN(get_logger(), "[Test] Behavior test #%d done", TEST_BEHAVIOR);
    }
    return;
#endif

    if (cur_stage_ >= 6) return;
    if (!stages_[cur_stage_]) return;  // Web模式跳过运动，无赛段

    stages_[cur_stage_]->run();

    // 统一下发参数（不再需要赛段-specific 分支）
    apply_stage_params();

    // 赛段切换
    if (stages_[cur_stage_]->is_done()) {
#ifdef DEBUG_STAGE
        RCLCPP_INFO(get_logger(), "[Stage] %d done", cur_stage_ + 1);
#endif
        if (single_stage_mode_) {
            motion_.stop();
            RCLCPP_WARN(get_logger(), "[DEBUG] Stage %d done, stopped", cur_stage_ + 1);
            timer_->cancel();
            return;
        }
#ifdef DEBUG_END_STAGE
        if (cur_stage_ + 1 >= DEBUG_END_STAGE) {
            motion_.stop();
            RCLCPP_WARN(get_logger(), "[DEBUG] Stage %d done, end stage mode", cur_stage_ + 1);
            timer_->cancel();
            return;
        }
#endif
        cur_stage_++;
        if (cur_stage_ < 6) {
#ifdef DEBUG_STAGE
            RCLCPP_INFO(get_logger(), "[Stage] switching to stage %d", cur_stage_ + 1);
#endif
            if (cur_stage_ == 2) lane_detector_.set_mode(LaneMode::RELAXED);
            else lane_detector_.set_mode(LaneMode::STRICT);
            if (cur_stage_ == 5) ball_detector_.reset_filter();
            stages_[cur_stage_]->init();
        } else {
            RCLCPP_INFO(get_logger(), "All stages complete!");
        }
    }

}

// ── 传感器回调 ──
void RaceController::on_rgb(sensor_msgs::msg::Image::SharedPtr msg) {
    try {
    // 统一转 BGR（检测器+调试画面全基于BGR，cv_bridge自动处理源编码rgb8→bgr8）
    auto cv_img = cv_bridge::toCvShare(msg, "bgr8");

    // ── 视觉检测（锁外，5-30ms，不阻塞 IMU/LiDAR 回调） ──
    auto lane  = lane_detector_.detect(cv_img->image);
    auto ball  = ball_detector_.detect(cv_img->image, BallColor::ORANGE);
    auto blue  = ball_detector_.detect(cv_img->image, BallColor::BLUE);
    auto white = ball_detector_.detect(cv_img->image, BallColor::WHITE);
    Stage4Result s4_result;
    if (cur_stage_ == 3) {
        s4_result = stage4_detector_.detect(cv_img->image);
    }

    // ── 写入共享数据（锁内，<0.01ms） ──
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        sensor_.lane_offset       = lane.offset;
        sensor_.lane_curvature    = lane.curvature;
        sensor_.lane_valid        = lane.valid;
        sensor_.lane_both_sides   = lane.both_sides;
        sensor_.ball_found        = ball.found;
        sensor_.ball_x            = ball.cx;
        sensor_.ball_dist         = ball.dist_m;
        sensor_.blue_ball_found   = blue.found;
        sensor_.blue_ball_x       = blue.cx;
        sensor_.blue_ball_dist    = blue.dist_m;
        sensor_.white_ball_found  = white.found;
        sensor_.white_ball_x      = white.cx;
        sensor_.white_ball_dist   = white.dist_m;
        // NOTE: vision_result 由本回调线程写入，由 stage4::run()（timer线程）读取，
        // 无锁访问。Stage4Result 是多字段 struct，理论上存在数据竞争，但 run() 为低频
        // 检查（~5Hz），实际使用中先到先得即可，不影响控制决策正确性。
        if (cur_stage_ == 3 && stages_[3]) {
            auto s4 = static_cast<Stage4*>(stages_[3].get());
            s4->vision_result = s4_result;
        }
    }

    // ── Web 推流（锁外，JPEG编码 20-40ms） ──
#ifdef ENABLE_WEB_STREAMING
    web_streamer_.push_frame(cv_img->image);
    int eo = web_streamer_.exposure_offset();
    if (eo < 0) {
        cv::Mat dark;
        cv_img->image.convertTo(dark, -1, 1.0, eo);
        web_streamer_.push_dark_frame(dark);
    } else {
        web_streamer_.push_dark_frame(cv_img->image);
    }
#endif

// ── 标注画面生成（供 DEBUG_VISION imshow 和 Web 双流共用） ──
#if defined(DEBUG_VISION) || defined(ENABLE_WEB_STREAMING)
    cv::Mat frame = cv_img->image.clone();
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);   // 统一BGR输入
    cv::inRange(hsv, cv::Scalar(20, 100, 150), cv::Scalar(35, 255, 255), mask);
    cv::Mat overlay = frame.clone();
    overlay.setTo(cv::Scalar(0, 255, 0), mask);
    cv::addWeighted(frame, 0.7, overlay, 0.3, 0, frame);

    for (auto& p : lane_detector_.last_left_)
        cv::circle(frame, p, 4, {255, 0, 0}, -1);
    for (auto& p : lane_detector_.last_right_)
        cv::circle(frame, p, 4, {0, 0, 255}, -1);

    auto& lpts = lane_detector_.last_left_;
    auto& rpts = lane_detector_.last_right_;
    for (size_t li = 0, ri = 0; li < lpts.size() && ri < rpts.size(); ) {
        if (lpts[li].y == rpts[ri].y) {
            cv::circle(frame, {(lpts[li].x + rpts[ri].x) / 2, lpts[li].y}, 3, {0, 255, 255}, -1);
            li++; ri++;
        } else if (lpts[li].y > rpts[ri].y) { li++; } else { ri++; }
    }

    if (lane.valid && lane.lane_width > 0) {
        if (lpts.empty() && !rpts.empty()) {
            for (auto& p : rpts) {
                int est_x = static_cast<int>(p.x - lane.lane_width);
                if (est_x >= 0) cv::circle(frame, {est_x, p.y}, 3, {255, 0, 255}, -1);
            }
        } else if (rpts.empty() && !lpts.empty()) {
            for (auto& p : lpts) {
                int est_x = static_cast<int>(p.x + lane.lane_width);
                if (est_x < frame.cols) cv::circle(frame, {est_x, p.y}, 3, {255, 0, 255}, -1);
            }
        }
    }

    if (ball.found) {
        int bx = static_cast<int>((ball.cx + 1.0f) / 2.0f * frame.cols);
        int by = static_cast<int>((ball.cy + 1.0f) / 2.0f * frame.rows);
        cv::circle(frame, {bx, by}, static_cast<int>(ball.radius), {0, 165, 255}, 2);
        cv::putText(frame, cv::format("r=%.0fpx d=%.2fm", ball.radius, ball.dist_m),
                    {bx + 5, by - 5}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 165, 255}, 1);
    }

    cv::line(frame, {frame.cols/2, 0}, {frame.cols/2, frame.rows}, {255, 255, 255}, 1);
    cv::putText(frame, cv::format("S%d | off=%.2f curv=%.1f ball=%d r=%.0fpx dist=%.2fm",
                cur_stage_+1, lane.offset, lane.curvature, ball.found, ball.radius, ball.dist_m),
            {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 255}, 2);
    // 快照 odom 字段用于显示（避免与回调线程数据竞争）
    float dox, doy, dyaw;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        dox = sensor_.odom_x; doy = sensor_.odom_y; dyaw = sensor_.yaw;
    }
    cv::putText(frame, cv::format("odom x=%.3f y=%.3f yaw=%.3f", dox, doy, dyaw),
            {10, 58}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 255}, 2);

    if (cur_stage_ == 3 && stages_[3]) {
        auto s4 = static_cast<Stage4*>(stages_[3].get());
        auto& r = s4->vision_result;
        if (r.football_found) {
            int fx = static_cast<int>((r.football_cx + 1.0f) / 2.0f * frame.cols);
            int fy = frame.rows / 2;
            cv::circle(frame, {fx, fy}, 20, {255, 255, 255}, 2);
            cv::line(frame, {fx, 0}, {fx, frame.rows}, {255, 255, 255}, 1);
            cv::putText(frame, cv::format("football d=%.2fm", r.football_dist),
                        {fx + 5, fy - 25}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 2);
        }
        int line_y = 86;
        auto draw_status = [&](bool found, float dist, const std::string& label, cv::Scalar color) {
            cv::putText(frame, cv::format("%s:%s d=%.2f", label.c_str(), found ? "Y" : "N", dist),
                        {10, line_y}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        found ? color : cv::Scalar(100, 100, 100), 1);
            line_y += 22;
        };
        draw_status(r.football_found, r.football_dist, "football", {255, 255, 255});
        draw_status(r.limbar_found,   r.limbar_dist,   "limbar",   {180, 180, 180});
        draw_status(r.coke_found,     r.coke_dist,     "coke",     {80,  80,  80 });
        draw_status(r.obstacle_found, r.obstacle_dist, "obstacle", {255, 128, 0  });
    }

#ifdef DEBUG_VISION
    cv::imshow("RaceDebug", frame);
    cv::waitKey(1);
#endif
#ifdef ENABLE_WEB_STREAMING
    web_streamer_.push_debug_frame(frame);
#endif
#endif  // defined(DEBUG_VISION) || defined(ENABLE_WEB_STREAMING)
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[RGB] cv_bridge error: %s (enc=%s)", e.what(), msg->encoding.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[RGB] error: %s", e.what());
    }
}

// ── D430i 左目红外回调（D430i 无RGB，只有红外+深度，红外最接近"相机画面"） ──
void RaceController::on_d435_infra1(sensor_msgs::msg::Image::SharedPtr msg) {
    (void)msg;
#ifdef ENABLE_WEB_STREAMING
    try {
        cv::Mat frame;
        if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
            auto cv_img = cv_bridge::toCvShare(msg, "mono8");
            cv::cvtColor(cv_img->image, frame, cv::COLOR_GRAY2BGR);
        } else if (msg->encoding == sensor_msgs::image_encodings::MONO16 || msg->encoding == "16UC1") {
            auto cv_img = cv_bridge::toCvShare(msg, "mono16");
            cv::Mat u8;
            cv_img->image.convertTo(u8, CV_8UC1, 1.0 / 256.0);
            cv::cvtColor(u8, frame, cv::COLOR_GRAY2BGR);
        } else {
            auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
            if (cv_img->image.channels() == 1) cv::cvtColor(cv_img->image, frame, cv::COLOR_GRAY2BGR);
            else frame = cv_img->image;
        }
        web_streamer_.push_d435_frame(frame);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "[D435 infra] cv_bridge error: %s", e.what());
    }
#endif
}

// ── D430i 深度图回调（mono16 mm → 归一化+JET伪彩色 → web展示） ──
void RaceController::on_d435_depth(sensor_msgs::msg::Image::SharedPtr msg) {
    (void)msg;
#ifdef ENABLE_WEB_STREAMING
    try {
        cv::Mat cv_depth;
        if (msg->encoding == "16UC1" || msg->encoding == "mono16") {
            cv_depth = cv::Mat(msg->height, msg->width, CV_16UC1,
                               const_cast<unsigned char*>(msg->data.data()), msg->step)
                           .clone();
        } else if (msg->encoding == sensor_msgs::image_encodings::MONO16) {
            auto cv_ptr = cv_bridge::toCvShare(msg, "mono16");
            cv_depth = cv_ptr->image.clone();
        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            auto cv_ptr = cv_bridge::toCvShare(msg, "32FC1");
            cv_ptr->image.convertTo(cv_depth, CV_16UC1, 1.0);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[D435 depth] unknown encoding: %s", msg->encoding.c_str());
            return;
        }
        cv::Mat norm, color;
        cv_depth.convertTo(norm, CV_8UC1, 255.0 / 5000.0);
        cv::applyColorMap(norm, color, cv::COLORMAP_JET);
        web_streamer_.push_depth_frame(color);
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[D435 depth] error: %s (encoding: %s)", e.what(), msg->encoding.c_str());
    }
#endif
}

// ── D430i 右目红外回调（mono8 → 灰度 → web展示，2026-08-06 接入） ──
void RaceController::on_d435_infra2(sensor_msgs::msg::Image::SharedPtr msg) {
    (void)msg;
#ifdef ENABLE_WEB_STREAMING
    try {
        cv::Mat frame;
        if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
            auto cv_img = cv_bridge::toCvShare(msg, "mono8");
            cv::cvtColor(cv_img->image, frame, cv::COLOR_GRAY2BGR);
        } else {
            auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
            if (cv_img->image.channels() == 1) cv::cvtColor(cv_img->image, frame, cv::COLOR_GRAY2BGR);
            else frame = cv_img->image;
        }
        web_streamer_.push_infra2_frame(frame);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "[D435 infra2] cv_bridge error: %s", e.what());
    }
#endif
}

#ifdef REAL_DOG
// ── TOF 头×2 高程（8x8=64点, 单位m, 取最低点 → tof_clearance） ──
void RaceController::on_tof_head(protocol::msg::HeadTofPayload::SharedPtr msg) {
    float min_h = 0.66f;
    bool  avail = false;
    for (const auto* tof : {&msg->left_head, &msg->right_head}) {
        if (!tof->data_available) continue;
        avail = true;
        for (float v : tof->data) if (v > 0.001f && v < min_h) min_h = v;
    }
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    sensor_.tof_clearance = min_h;
    sensor_.tof_available = avail;
}

// ── TOF 尾×2 高程 ──
void RaceController::on_tof_rear(protocol::msg::RearTofPayload::SharedPtr msg) {
    float min_h = 0.66f;
    bool  avail = false;
    for (const auto* tof : {&msg->left_rear, &msg->right_rear}) {
        if (!tof->data_available) continue;
        avail = true;
        for (float v : tof->data) if (v > 0.001f && v < min_h) min_h = v;
    }
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (min_h < sensor_.tof_clearance) sensor_.tof_clearance = min_h;
    sensor_.tof_available = sensor_.tof_available || avail;
}

// ── 超声测距 ──
void RaceController::on_ultrasonic(sensor_msgs::msg::Range::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    sensor_.ultrasonic_range = msg->range;
}
#endif

void RaceController::on_sim_state(const lcm::ReceiveBuffer*, const std::string&,
                                  const simulator_lcmt* msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    sensor_.odom_x = static_cast<float>(msg->p[0]);
    sensor_.odom_y = static_cast<float>(msg->p[1]);
    sensor_.body_height = static_cast<float>(msg->p[2]);
}

#ifdef REAL_DOG
// ── 真机 LCM 里程计回调（global_to_robot → localization_lcmt, 50Hz, 7667） ──
void RaceController::on_global_to_robot(const lcm::ReceiveBuffer*,
                                         const std::string&,
                                         const localization_lcmt* msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    sensor_.odom_x = msg->xyz[0];
    sensor_.odom_y = msg->xyz[1];
    sensor_.body_height = msg->xyz[2];  // z 轴直接作为身高
    sensor_.abs_yaw = msg->rpy[2];      // 地图坐标系绝对朝向（供绝对转向, 2026-08-07）
    sensor_.pitch_map = msg->rpy[1];    // 地图坐标系俯仰（供抬头/姿态验证, 2026-08-08）
    sensor_.roll_map = msg->rpy[0];     // 地图坐标系横滚/侧倾（供身躯倾斜验证, 2026-08-08）
}

// ── 真机 LCM 状态估计回调（state_estimator，备选 body_height 来源） ──
void RaceController::on_state_estimator(const lcm::ReceiveBuffer*,
                                         const std::string&,
                                         const state_estimator_lcmt* msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    // 如果 global_to_robot.xyz[2] 不够精确，可用 state_estimator.p[2] 覆盖
    sensor_.body_height = msg->p[2];
}

// ── 真机 BMS 电池监控（低电量自动保护） ──
void RaceController::on_bms(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // TODO: 拿到 protocol::msg::BmsStatus 后替换为 batt_soc 字段
    if (msg->data.empty()) return;
    float soc = msg->data[0];  // 临时假设 data[0]=batt_soc 电量百分比
    if (soc < 20.f) {
        RCLCPP_WARN(get_logger(), "[BMS] Battery %.0f%% — EMERGENCY STOP + LIE DOWN", soc);
        motion_.stop();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        motion_.lie_down();
        timer_->cancel();
    } else if (soc < 30.f) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "[BMS] Battery low: %.0f%%", soc);
    }
}

// ── 真机触摸紧急停止（TODO：需 protocol::msg::TouchStatus 类型定义） ──
// touch_status topic 格式: std_msgs/Header header + int32 touch_state + uint64 timestamp
// touch_state: 0x01=单击 0x03=双击 0x07=长按(LPWG_TOUCHANDHOLD_DETECTED)
// void RaceController::on_touch(protocol::msg::TouchStatus::SharedPtr msg) {
//     if (msg->touch_state == 0x07) {
//         RCLCPP_WARN(get_logger(), "[Touch] Long press — EMERGENCY STOP");
//         motion_.stop();
//         rclcpp::sleep_for(std::chrono::milliseconds(200));
//         motion_.lie_down();
//     }
// }
#endif

void RaceController::on_imu(sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    auto& q = msg->orientation;
    sensor_.yaw   = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                               1.0 - 2.0*(q.y*q.y + q.z*q.z));
    sensor_.pitch = std::asin(2.0*(q.w*q.y - q.z*q.x));
    sensor_.roll  = std::atan2(2.0*(q.w*q.x + q.y*q.z),
                               1.0 - 2.0*(q.x*q.x + q.y*q.y));
}

void RaceController::on_lidar(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // ⚠ 真狗 LiDAR 可能走 sensor_manager 的 ScanMsg 而非标准 LaserScan
    //    上机后若收不到数据，先 ros2 topic info /scan 确认 type
    //    如果是自定义 ScanMsg，需替换消息类型并适配字段映射
    if (msg->ranges.empty()) return;
    int n = msg->ranges.size();
    float front_min = 10.0f;
    for (int i = n/3; i < 2*n/3; i++) {
        if (msg->ranges[i] < front_min) front_min = msg->ranges[i];
    }
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        sensor_.lidar_front = front_min;
    }

#ifdef ENABLE_WEB_STREAMING
    render_lidar_frame(msg->ranges, msg->angle_min, msg->angle_increment, front_min);
#endif
}

// ═══════════════════════════════════════════════════════
// Web 仪表盘渲染方法（从 control_loop / on_lidar 抽取）
// ═══════════════════════════════════════════════════════
#ifdef ENABLE_WEB_STREAMING

void RaceController::render_track_frame() {
    // 快照传感器字段（避免与回调线程数据竞争）
    float ox, oy, yw;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        ox = sensor_.odom_x; oy = sensor_.odom_y; yw = sensor_.yaw;
    }

    const int SIZE = 240;
    cv::Mat track_img(SIZE, SIZE, CV_8UC3, cv::Scalar(10, 15, 30));

    if (odom_history_.size() >= 2) {
        float min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
        for (auto& p : odom_history_) {
            if (p.first < min_x) min_x = p.first;
            if (p.first > max_x) max_x = p.first;
            if (p.second < min_y) min_y = p.second;
            if (p.second > max_y) max_y = p.second;
        }
        float range = std::max(max_x - min_x, max_y - min_y);
        if (range < 0.5f) range = 2.0f;
        float pad = range * 0.15f;
        min_x -= pad; max_x += pad; min_y -= pad; max_y += pad;
        range = std::max(max_x - min_x, max_y - min_y);

        float scale_x = (SIZE - 30) / range;
        float scale_y = (SIZE - 30) / range;
        int off_x = 15, off_y = SIZE - 15;

        auto to_px = [&](float wx, float wy) {
            return cv::Point(off_x + (wx - min_x) * scale_x,
                             off_y - (wy - min_y) * scale_y);
        };

        // ── 网格（自适应 0.5/1/2m 步长，估算距离用） ──
        float grid = 0.5f;
        int g = static_cast<int>(grid * scale_x);
        if (g < 12) { grid = 1.0f; g = static_cast<int>(grid * scale_x); }
        if (g < 12) { grid = 2.0f; g = static_cast<int>(grid * scale_x); }
        for (float wx = std::floor(min_x / grid) * grid; wx <= max_x; wx += grid) {
            int px = to_px(wx, 0).x;
            cv::line(track_img, {px, 10}, {px, SIZE - 10}, {25, 30, 45}, 1);
        }
        for (float wy = std::floor(min_y / grid) * grid; wy <= max_y; wy += grid) {
            int py = to_px(0, wy).y;
            cv::line(track_img, {10, py}, {SIZE - 10, py}, {25, 30, 45}, 1);
        }
        cv::putText(track_img, cv::format("%.1fm", grid), {SIZE - 40, SIZE - 8},
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, {80, 90, 110}, 1);

        // ── 轨迹 ──
        for (size_t i = 1; i < odom_history_.size(); i++) {
            float t = static_cast<float>(i) / odom_history_.size();
            cv::line(track_img, to_px(odom_history_[i-1].first, odom_history_[i-1].second),
                     to_px(odom_history_[i].first, odom_history_[i].second),
                     cv::Scalar(50.0 + 155.0*t, 100.0+155.0*t, 255.0), 2);
        }

        // ── 起点标记 ──
        auto start = to_px(odom_history_.front().first, odom_history_.front().second);
        cv::circle(track_img, start, 4, {0, 255, 0}, -1);
        cv::putText(track_img, "S", {start.x - 4, start.y - 8},
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 255, 0}, 1);

        // ── 当前位置 + 朝向箭头 ──
        auto cur = to_px(ox, oy);
        cv::circle(track_img, cur, 6, {0, 200, 255}, -1);
        float arrow_len = 18.0f;
        cv::Point tip(cur.x + arrow_len * std::cos(yw),
                      cur.y - arrow_len * std::sin(yw));
        cv::arrowedLine(track_img, cur, tip, {0, 200, 255}, 2);

        // ── 信息：坐标 / 距起点 / 航向 / 点数 ──
        float dx = ox - odom_history_.front().first;
        float dy = oy - odom_history_.front().second;
        float dist_from_start = std::sqrt(dx*dx + dy*dy);
        cv::putText(track_img, cv::format("x:%.2f y:%.2f", ox, oy),
                    {5, 15}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 255, 100}, 1);
        cv::putText(track_img, cv::format("dist:%.1fm hdg:%ddeg", dist_from_start,
                    static_cast<int>(yw * 180 / M_PI)),
                    {5, 33}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {120, 140, 120}, 1);
        cv::putText(track_img, cv::format("pts:%zu", odom_history_.size()),
                    {5, 51}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {100, 120, 100}, 1);
    } else {
        cv::putText(track_img, "waiting for odom...", {30, SIZE/2},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {100, 100, 100}, 1);
    }

    web_streamer_.push_track_frame(track_img);
}

void RaceController::render_telemetry_frame() {
    // 快照传感器字段（避免与回调线程数据竞争）
    float bh, sp, sr, sy, lf, tof, ultra;
    bool b_found; float b_dist;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        bh = sensor_.body_height; sp = sensor_.pitch; sr = sensor_.roll;
        sy = sensor_.yaw; lf = sensor_.lidar_front;
        tof = sensor_.tof_clearance; ultra = sensor_.ultrasonic_range;
        b_found = sensor_.ball_found; b_dist = sensor_.ball_dist;
    }

    const int TW = 360, TH = 300;
    cv::Mat telem(TH, TW, CV_8UC3, cv::Scalar(10, 15, 30));

    int y = 18;
    auto row = [&](const std::string& label, const std::string& val, cv::Scalar vc = {0,255,100}) {
        cv::putText(telem, label, {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {180,180,200}, 1);
        cv::putText(telem, val, {150, y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, vc, 1);
        y += 22;
    };
    row("Stage", cv::format("%d/6", cur_stage_ + 1), {233, 69, 96});
    row("Height", cv::format("%.2f m", bh));
    row("StepH", cv::format("%.2f m", last_sent_step_h_));
    row("pitch/roll", cv::format("%.2f / %.2f rad", sp, sr));
    row("yaw", cv::format("%.0f deg", sy * 180 / M_PI));
    // TOF 离地间隙（独木桥关键：<0.15m 红警）
    row("TOF", cv::format("%.2f m", tof),
        tof < 0.15f ? cv::Scalar{0,0,255} : cv::Scalar{0,255,100});
    // 超声（0=无数据，-- 显示）
    row("Ultra", ultra > 0.01f ? cv::format("%.2f m", ultra) : "--",
        (ultra > 0.01f && ultra < 0.5f) ? cv::Scalar{0,255,255} : cv::Scalar{0,255,100});
    // Lidar 前方最近障碍
    row("Lidar", cv::format("%.2f m", lf),
        lf < 1.0f ? cv::Scalar{0,0,255} : cv::Scalar{0,255,100});
    // 目标球检测
    row("Ball", b_found ? cv::format("%.2f m", b_dist) : "none",
        b_found ? cv::Scalar{0,255,0} : cv::Scalar{150,150,150});
    y += 4;

    // ── 身高条 ──
    cv::putText(telem, "Height", {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {180,180,200}, 1);
    int bar_x = 80, bar_w = 200, bar_h = 12, bar_y = y - 10;
    cv::rectangle(telem, {bar_x, bar_y}, {bar_x + bar_w, bar_y + bar_h}, {60,60,80}, 1);
    float h_ratio = std::min(bh / 0.5f, 1.0f);
    cv::rectangle(telem, {bar_x, bar_y},
                  {bar_x + static_cast<int>(bar_w * h_ratio), bar_y + bar_h},
                  {0, 180, 100}, -1);
    cv::putText(telem, cv::format("%.2f/0.50m", bh),
                {bar_x + bar_w + 5, y}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {150,150,160}, 1);
    y += 22;

    // ── TOF 离地条（独木桥：<0.15 红警） ──
    cv::putText(telem, "TOF", {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {180,180,200}, 1);
    int tbar_x = 80, tbar_w = 200, tbar_h = 12, tbar_y = y - 10;
    cv::rectangle(telem, {tbar_x, tbar_y}, {tbar_x + tbar_w, tbar_y + tbar_h}, {60,60,80}, 1);
    float t_ratio = std::min(tof / 0.66f, 1.0f);
    cv::rectangle(telem, {tbar_x, tbar_y},
                  {tbar_x + static_cast<int>(tbar_w * t_ratio), tbar_y + tbar_h},
                  tof < 0.15f ? cv::Scalar{0,0,255} : cv::Scalar{0,200,180}, -1);
    cv::putText(telem, cv::format("%.2f/0.66m", tof),
                {tbar_x + tbar_w + 5, y}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {150,150,160}, 1);
    y += 24;

    // ── yaw 罗盘（四向刻度） ──
    cv::putText(telem, "Compass", {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {180,180,200}, 1);
    int comp_cx = TW - 60, comp_cy = y + 24, comp_r = 32;
    cv::circle(telem, {comp_cx, comp_cy}, comp_r, {60,60,80}, 1);
    for (int d = 0; d < 4; d++) {
        double a = d * M_PI / 2;
        cv::Point p1(comp_cx + static_cast<int>((comp_r - 6) * std::cos(a)),
                     comp_cy - static_cast<int>((comp_r - 6) * std::sin(a)));
        cv::Point p2(comp_cx + static_cast<int>((comp_r + 4) * std::cos(a)),
                     comp_cy - static_cast<int>((comp_r + 4) * std::sin(a)));
        cv::line(telem, p1, p2, {90, 90, 110}, 1);
    }
    cv::Point arrow_tip(comp_cx + comp_r * std::cos(sy),
                        comp_cy - comp_r * std::sin(sy));
    cv::arrowedLine(telem, {comp_cx, comp_cy}, arrow_tip, {0, 200, 255}, 2);
    cv::putText(telem, "N", {comp_cx - 6, comp_cy - comp_r - 5},
                cv::FONT_HERSHEY_SIMPLEX, 0.35, {120,120,140}, 1);
    cv::putText(telem, "E", {comp_cx + comp_r - 14, comp_cy + 4},
                cv::FONT_HERSHEY_SIMPLEX, 0.35, {120,120,140}, 1);
    cv::putText(telem, "S", {comp_cx - 6, comp_cy + comp_r + 15},
                cv::FONT_HERSHEY_SIMPLEX, 0.35, {120,120,140}, 1);
    cv::putText(telem, "W", {comp_cx - comp_r + 2, comp_cy + 4},
                cv::FONT_HERSHEY_SIMPLEX, 0.35, {120,120,140}, 1);
    y += 62;

    row("RC", last_rc_mode_ ? "ON" : "OFF", last_rc_mode_ ? cv::Scalar{0,255,0} : cv::Scalar{150,150,150});

    web_streamer_.push_telemetry_frame(telem);
}

void RaceController::render_lidar_frame(const std::vector<float>& ranges,
                                         float angle_min, float angle_inc,
                                         float front_min) {
    const int SIZE = 240;
    const float MAX_RANGE = 8.0f;
    const float SCALE = (SIZE / 2) / MAX_RANGE;
    cv::Mat lidar_img(SIZE, SIZE, CV_8UC3, cv::Scalar(10, 15, 30));

    int cx = SIZE / 2, cy = SIZE - 20;

    for (int r = 1; r <= 4; r++) {
        int pr = static_cast<int>(r * 2.0f * SCALE);
        cv::circle(lidar_img, {cx, cy}, pr, {50, 50, 70}, 1);
        cv::putText(lidar_img, std::to_string(r * 2) + "m",
                    {cx + pr - 15, cy - 5}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {70, 70, 90}, 1);
    }

    int num = ranges.size();
    float angle_max = angle_min + num * angle_inc;
    for (int i = 0; i < num; i++) {
        float dist = ranges[i];
        if (dist < 0.1f || dist > MAX_RANGE) continue;
        float angle = angle_min + i * angle_inc;
        int px = cx + static_cast<int>(dist * std::sin(angle) * SCALE);
        int py = cy - static_cast<int>(dist * std::cos(angle) * SCALE);
        if (px < 0 || px >= SIZE || py < 0 || py >= SIZE) continue;

        float ratio = std::min(dist / MAX_RANGE, 1.0f);
        cv::Vec3b color(0, static_cast<uint8_t>(255 * (1 - ratio)),
                          static_cast<uint8_t>(100 + 155 * ratio));
        cv::circle(lidar_img, {px, py}, 2, color, -1);
    }

    cv::circle(lidar_img, {cx, cy}, 8, {0, 200, 255}, -1);
    cv::line(lidar_img, {cx, cy}, {cx, cy - 20}, {0, 200, 255}, 2);

    cv::putText(lidar_img, cv::format("front: %.2fm", front_min),
                {5, 15}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 100}, 1);
    cv::putText(lidar_img, cv::format("samples: %d  FOV: %.0f deg",
                num, (angle_max - angle_min) * 180.0f / M_PI),
                {5, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {120, 120, 140}, 1);

    web_streamer_.push_lidar_frame(lidar_img);
}

#endif  // ENABLE_WEB_STREAMING
