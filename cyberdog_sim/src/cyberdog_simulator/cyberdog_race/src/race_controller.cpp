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
    sub_d435_ = create_subscription<sensor_msgs::msg::Image>(
        TOPIC_D435, qos_be,
        [this](sensor_msgs::msg::Image::SharedPtr msg) { on_d435(msg); });

#ifdef REAL_DOG
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        TOPIC_ODOM, 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });
#endif

    yaml_pub_ = create_publisher<cyberdog_msg::msg::YamlParam>("yaml_parameter", 10);
    auto param = cyberdog_msg::msg::YamlParam();
    param.name = "use_rc";
    param.kind = 2;
    param.s64_value = 0;
    param.is_user = 0;
    rclcpp::sleep_for(std::chrono::seconds(1));
    yaml_pub_->publish(param);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    motion_.recovery();
    rclcpp::sleep_for(std::chrono::seconds(2));
    motion_.locomotion();
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    motion_.set_pitch(-0.26f);

    stages_[0] = std::make_unique<Stage1>(motion_, sensor_);
    stages_[1] = std::make_unique<Stage2>(motion_, sensor_);
    stages_[2] = std::make_unique<Stage3>(motion_, sensor_);
    stages_[3] = std::make_unique<Stage4>(motion_, sensor_);
    stages_[4] = std::make_unique<Stage5>(motion_, sensor_);
    stages_[5] = std::make_unique<Stage6>(motion_, sensor_);

    if (cur_stage_ == 2) lane_detector_.set_mode(LaneMode::RELAXED);
    if (cur_stage_ == 5) ball_detector_.reset_filter();
    stages_[cur_stage_]->init();

    if (lcm_sub_.good()) {
#ifdef REAL_DOG
        lcm_sub_.subscribe(LCM_STATE_ESTIMATOR,
                           &RaceController::on_state_estimator, this);
        RCLCPP_INFO(get_logger(), "[RealDog] Using LCM %s + ROS2 %s for odom",
                    LCM_STATE_ESTIMATOR, TOPIC_ODOM);
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
    // ── 真机：同时通过 LCM control_parameter 通道下发参数 ──
    // TODO: 确认真狗 motion 模块接受 LCM control_parameter_lcmt 还是 exec_request
    //       如果真狗只走 LCM，需用 control_parameter_lcmt 替代 ROS2 yaml_parameter
    // {
    //     control_parameter_request_lcmt req;
    //     snprintf(req.name, sizeof(req.name), "des_roll_pitch_height");
    //     snprintf(req.value, sizeof(req.value), "%.3f,%.3f,%.3f", roll, 0.0, h);
    //     req.parameterKind = 3;
    //     lcm_sub_.publish(LCM_CMD_EXEC, &req);
    // }
#endif
}

// ── 100Hz 控制循环 ──
void RaceController::control_loop() {
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

#ifdef ENABLE_WEB_STREAMING
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        odom_history_.emplace_back(sensor_.odom_x, sensor_.odom_y);
    }
    if (odom_history_.size() > 400) odom_history_.pop_front();
    if (++track_render_counter_ >= 20) { track_render_counter_ = 0; render_track_frame(); }
    if (++telem_render_counter_ >= 20) { telem_render_counter_ = 0; render_telemetry_frame(); }
#endif
}

// ── 传感器回调 ──
void RaceController::on_rgb(sensor_msgs::msg::Image::SharedPtr msg) {
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
        if (cur_stage_ == 3) {
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
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
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

    if (cur_stage_ == 3) {
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
}

// ── D435 深度相机回调 ──
void RaceController::on_d435(sensor_msgs::msg::Image::SharedPtr msg) {
    (void)msg;
#ifdef ENABLE_WEB_STREAMING
    auto cv_img = cv_bridge::toCvShare(msg, "bgr8");
    web_streamer_.push_d435_frame(cv_img->image);
#endif
}

void RaceController::on_sim_state(const lcm::ReceiveBuffer*, const std::string&,
                                  const simulator_lcmt* msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    sensor_.odom_x = static_cast<float>(msg->p[0]);
    sensor_.odom_y = static_cast<float>(msg->p[1]);
    sensor_.body_height = static_cast<float>(msg->p[2]);
}

#ifdef REAL_DOG
// ── 真机 ROS2 里程计回调 ──
void RaceController::on_odom(nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    sensor_.odom_x = static_cast<float>(msg->pose.pose.position.x);
    sensor_.odom_y = static_cast<float>(msg->pose.pose.position.y);
    // body_height 从 LCM state_estimator 获取，此处不更新
}

// ── 真机 LCM 状态估计回调 ──
void RaceController::on_state_estimator(const lcm::ReceiveBuffer*,
                                         const std::string&,
                                         const state_estimator_lcmt* msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    // p[2] = z 方向绝对位置，可作为 body_height 参考
    // TODO: 真机验证 body_height 是否需要减去地面高度偏移
    sensor_.body_height = msg->p[2];
    // 如果 ROS2 /odom 不可用，可启用下面两行作为备选里程计
    // sensor_.odom_x = msg->p[0];
    // sensor_.odom_y = msg->p[1];
}
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

        for (size_t i = 1; i < odom_history_.size(); i++) {
            float t = static_cast<float>(i) / odom_history_.size();
            cv::line(track_img, to_px(odom_history_[i-1].first, odom_history_[i-1].second),
                     to_px(odom_history_[i].first, odom_history_[i].second),
                     cv::Scalar(50.0 + 155.0*t, 100.0+155.0*t, 255.0), 2);
        }

        auto cur = to_px(ox, oy);
        cv::circle(track_img, cur, 6, {0, 200, 255}, -1);
        float arrow_len = 18.0f;
        cv::Point tip(cur.x + arrow_len * std::cos(yw),
                      cur.y - arrow_len * std::sin(yw));
        cv::arrowedLine(track_img, cur, tip, {0, 200, 255}, 2);

        cv::putText(track_img, cv::format("x:%.2f y:%.2f", ox, oy),
                    {5, 15}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 255, 100}, 1);
        cv::putText(track_img, cv::format("pts:%zu", odom_history_.size()),
                    {5, 33}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {120, 140, 120}, 1);
    } else {
        cv::putText(track_img, "waiting for odom...", {30, SIZE/2},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {100, 100, 100}, 1);
    }

    web_streamer_.push_track_frame(track_img);
}

void RaceController::render_telemetry_frame() {
    // 快照传感器字段（避免与回调线程数据竞争）
    float bh, sp, sr, sy, lf;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        bh = sensor_.body_height; sp = sensor_.pitch; sr = sensor_.roll;
        sy = sensor_.yaw; lf = sensor_.lidar_front;
    }

    const int TW = 320, TH = 240;
    cv::Mat telem(TH, TW, CV_8UC3, cv::Scalar(10, 15, 30));

    int y = 18;
    auto row = [&](const std::string& label, const std::string& val, cv::Scalar vc = {0,255,100}) {
        cv::putText(telem, label, {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {180,180,200}, 1);
        cv::putText(telem, val, {140, y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, vc, 1);
        y += 22;
    };
    row("赛段", cv::format("%d/6", cur_stage_ + 1), {233, 69, 96});
    row("身高", cv::format("%.2f m", bh));
    row("步高上限", cv::format("%.2f m", last_sent_step_h_));
    row("pitch", cv::format("%.3f rad", sp));
    row("roll",  cv::format("%.3f rad", sr));
    row("yaw",   cv::format("%.2f (%.0f deg)", sy, sy * 180/M_PI));
    row("lidar front", cv::format("%.2f m", lf),
        lf < 1.0f ? cv::Scalar{0,0,255} : cv::Scalar{0,255,100});
    y += 6;

    cv::putText(telem, "身高", {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {180,180,200}, 1);
    int bar_x = 70, bar_w = 180, bar_h = 12, bar_y = y - 10;
    cv::rectangle(telem, {bar_x, bar_y}, {bar_x + bar_w, bar_y + bar_h}, {60,60,80}, 1);
    float h_ratio = std::min(bh / 0.5f, 1.0f);
    cv::rectangle(telem, {bar_x, bar_y},
                  {bar_x + static_cast<int>(bar_w * h_ratio), bar_y + bar_h},
                  {0, 180, 100}, -1);
    cv::putText(telem, cv::format("%.2f/0.50m", bh),
                {bar_x + bar_w + 5, y}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {150,150,160}, 1);
    y += 20;

    cv::putText(telem, "yaw罗盘", {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {180,180,200}, 1);
    int comp_cx = TW - 55, comp_cy = y + 18, comp_r = 28;
    cv::circle(telem, {comp_cx, comp_cy}, comp_r, {60,60,80}, 1);
    cv::Point arrow_tip(comp_cx + comp_r * std::cos(sy),
                        comp_cy - comp_r * std::sin(sy));
    cv::arrowedLine(telem, {comp_cx, comp_cy}, arrow_tip, {0, 200, 255}, 2);
    cv::putText(telem, "N", {comp_cx - 6, comp_cy - comp_r - 4},
                cv::FONT_HERSHEY_SIMPLEX, 0.35, {120,120,140}, 1);
    y += 50;

    row("RC模式", last_rc_mode_ ? "ON" : "OFF", last_rc_mode_ ? cv::Scalar{0,255,0} : cv::Scalar{150,150,150});

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
