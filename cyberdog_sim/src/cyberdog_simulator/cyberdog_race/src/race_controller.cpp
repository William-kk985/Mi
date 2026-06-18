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
        "/RGB_camera/image_raw", qos_be,
        [this](sensor_msgs::msg::Image::SharedPtr msg) { on_rgb(msg); });
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", qos_be,
        [this](sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });
    sub_lidar_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { on_lidar(msg); });

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
        lcm_sub_.subscribe("simulator_state", &RaceController::on_sim_state, this);
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

    RCLCPP_INFO(get_logger(), "Race controller started, stage %d", cur_stage_ + 1);
}

RaceController::~RaceController() {
    lcm_running_ = false;
    if (lcm_thread_.joinable()) lcm_thread_.join();
}

// ── 传感器快照 ──
SensorData RaceController::read_sensor_snapshot() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return sensor_;
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

    // 取传感器快照
    SensorData local = read_sensor_snapshot();
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        sensor_ = local;
    }

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
    auto cv_img = cv_bridge::toCvShare(msg, "bgr8");
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    auto lane = lane_detector_.detect(cv_img->image);
    auto ball = ball_detector_.detect(cv_img->image, BallColor::ORANGE);
    auto blue = ball_detector_.detect(cv_img->image, BallColor::BLUE);
    auto white = ball_detector_.detect(cv_img->image, BallColor::WHITE);
    sensor_.lane_offset     = lane.offset;
    sensor_.lane_curvature  = lane.curvature;
    sensor_.lane_valid      = lane.valid;
    sensor_.lane_both_sides = lane.both_sides;
    sensor_.ball_found  = ball.found;
    sensor_.ball_x      = ball.cx;
    sensor_.ball_dist   = ball.dist_m;
    sensor_.blue_ball_found = blue.found;
    sensor_.blue_ball_x     = blue.cx;
    sensor_.blue_ball_dist  = blue.dist_m;
    sensor_.white_ball_found = white.found;
    sensor_.white_ball_x     = white.cx;
    sensor_.white_ball_dist  = white.dist_m;

    // stage4 视觉检测
    if (cur_stage_ == 3) {
        auto s4 = static_cast<Stage4*>(stages_[3].get());
        s4->vision_result = stage4_detector_.detect(cv_img->image);
    }

#ifdef DEBUG_VISION
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
    cv::putText(frame, cv::format("odom x=%.3f y=%.3f yaw=%.3f",
                sensor_.odom_x, sensor_.odom_y, sensor_.yaw),
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

    cv::imshow("RaceDebug", frame);
    cv::waitKey(1);
#endif
}

void RaceController::on_sim_state(const lcm::ReceiveBuffer*, const std::string&,
                                  const simulator_lcmt* msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    sensor_.odom_x = static_cast<float>(msg->p[0]);
    sensor_.odom_y = static_cast<float>(msg->p[1]);
    sensor_.body_height = static_cast<float>(msg->p[2]);
}

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
}
