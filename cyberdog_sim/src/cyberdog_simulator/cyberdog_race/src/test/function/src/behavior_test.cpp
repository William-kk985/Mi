#include "behavior_test.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <cmath>

namespace behavior {

void run_test(MotionCtrl& motion, SensorData& sensor, int test_id) {
    fprintf(stderr, "\033[1;35m[BehaviorTest] Running #%d\033[0m\n", test_id);
    motion.locomotion();

    switch (test_id) {
        case 1:  jump_test(motion, sensor);        break;
        case 2:  scan_ball_test(motion, sensor);   break;
        case 3:  crouch_test(motion, sensor);      break;
        case 4:  navigate_test(motion, sensor);    break;
        case 5:  turn_test(motion, sensor);        break;
        case 6:  stand_lie_test(motion, sensor);   break;
        case 7:  pitch_test(motion, sensor);       break;
        case 8:  step_height_test(motion, sensor); break;
        case 9:  sensor_check_test(motion, sensor);break;
        case 10: rgb_view_test(motion, sensor);     break;
        case 11: march_in_place_test(motion, sensor); break;
        case 12: forward_test(motion, sensor);        break;
        case 13: jump30_test(motion, sensor);          break;
        case 14: turn_angle_test(motion, sensor);      break;
        case 15: abs_turn_test(motion, sensor);        break;
        default:
            fprintf(stderr, "\033[1;31m[BehaviorTest] Unknown #%d\033[0m\n", test_id);
            break;
    }
    fprintf(stderr, "\033[1;32m[BehaviorTest] #%d done\033[0m\n", test_id);
}

void jump_test(MotionCtrl& motion, SensorData& sensor) {
    (void)sensor;
    motion.stand();
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    for (int attempt = 0; attempt < 3; attempt++) {
        motion.jump();
        for (int i = 0; i < 250; i++) {
            motion.send_lcm_mode(12);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }
    motion.stop();
}

void scan_ball_test(MotionCtrl& motion, SensorData& sensor) {
    for (int i = 0; i < 200; i++) {
        motion.set_velocity(0.f, 0.f, i < 100 ? 0.3f : -0.3f);
        if (sensor.ball_found) {
            fprintf(stderr, "[Test] Ball found dist=%.2f\n", sensor.ball_dist);
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    motion.stop();
}

void crouch_test(MotionCtrl& motion, SensorData& sensor) {
    (void)motion; (void)sensor;
    fprintf(stderr, "[Test] see apply_stage_params\n");
}

void navigate_test(MotionCtrl& motion, SensorData& sensor) {
    float tx = 2.0f, ty = 3.0f;
    for (int i = 0; i < 500; i++) {
        float dx = tx - sensor.odom_x;
        float dy = ty - sensor.odom_y;
        float dist = std::sqrt(dx*dx + dy*dy);
        if (dist < 0.1f) break;

        float yaw = std::atan2(dy, dx) - sensor.yaw;
        while (yaw >  M_PI) yaw -= 2*M_PI;
        while (yaw < -M_PI) yaw += 2*M_PI;

        float yaw_cmd = std::max(-0.5f, std::min(0.5f, yaw * 2.0f));
        motion.set_velocity(std::min(0.5f, dist), 0.f, yaw_cmd);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    motion.stop();
}

void turn_test(MotionCtrl& motion, SensorData& sensor) {
    (void)sensor;
    for (int i = 0; i < 200; i++) {
        motion.set_velocity(0.f, 0.f, 0.5f);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    motion.stop();
}

void stand_lie_test(MotionCtrl& motion, SensorData& sensor) {
    (void)sensor;
    motion.stand();     rclcpp::sleep_for(std::chrono::seconds(1));
    motion.locomotion(); rclcpp::sleep_for(std::chrono::seconds(1));
    motion.stand();     rclcpp::sleep_for(std::chrono::seconds(1));
    motion.lie_down();  rclcpp::sleep_for(std::chrono::seconds(1));
}

void pitch_test(MotionCtrl& motion, SensorData& sensor) {
    (void)sensor;
    fprintf(stderr, "\033[1;35m[Pitch] 低头 15° 3秒...\033[0m\n");
    for (int i = 0; i < 60; i++) {
        motion.set_body_pitch(-0.26f);   // mode=21 真机姿态控制
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    fprintf(stderr, "\033[1;35m[Pitch] 回正 2秒...\033[0m\n");
    for (int i = 0; i < 40; i++) {
        motion.set_body_pitch(0.0f);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    fprintf(stderr, "\033[1;35m[Pitch] 抬头 15° 3秒...\033[0m\n");
    for (int i = 0; i < 60; i++) {
        motion.set_body_pitch(0.26f);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    motion.set_body_pitch(0.0f);
}

void step_height_test(MotionCtrl& motion, SensorData& sensor) {
    (void)sensor;
    motion.set_step_height(0.35f, 0.35f); rclcpp::sleep_for(std::chrono::seconds(1));
    motion.set_step_height(0.06f, 0.06f); rclcpp::sleep_for(std::chrono::seconds(1));
    motion.set_step_height(0.20f, 0.20f);
}

void sensor_check_test(MotionCtrl& motion, SensorData& sensor) {
    (void)motion;
    fprintf(stderr, "\033[1;35m========================================\033[0m\n");
    fprintf(stderr, "\033[1;35m  传感器链路检查（等待数据 5s）\033[0m\n");
    fprintf(stderr, "\033[1;35m========================================\033[0m\n");
    rclcpp::sleep_for(std::chrono::seconds(5));

    // 快照
    float odom_x, odom_y, yaw, body_h, lidar_f, pitch, roll;
    int lane_v, ball_f;
    {
        odom_x = sensor.odom_x; odom_y = sensor.odom_y;
        yaw = sensor.yaw; body_h = sensor.body_height;
        lidar_f = sensor.lidar_front;
        lane_v = sensor.lane_valid; ball_f = sensor.ball_found;
        pitch = sensor.pitch; roll = sensor.roll;
    }

    fprintf(stderr, "\n\033[1;36m  ✅ 里程计 odom_out\033[0m  x=%.3f y=%.3f yaw=%.2f h=%.3fm\n", odom_x, odom_y, yaw, body_h);
    fprintf(stderr, "\033[1;36m  ✅ LiDAR scan\033[0m        front=%.2fm (10=无近障)\n", lidar_f);
    fprintf(stderr, "\033[1;36m  ✅ IMU camera/imu\033[0m     pitch=%.2f roll=%.2f\n", pitch, roll);
    fprintf(stderr, "\033[1;36m  👁  RGB视觉\033[0m           lane=%d ball=%d\n", lane_v, ball_f);
    fprintf(stderr, "\n\033[1;32m  全部传感器已收集。红外/深度/BMS/TOF 由各自回调处理。\033[0m\n");
    fprintf(stderr, "\033[1;35m========================================\033[0m\n");
}

// ── 原地踏步（WALK_USERTROT + vel=0 → 原地小跑） ──
void march_in_place_test(MotionCtrl& motion, SensorData& sensor) {
    (void)sensor;
    fprintf(stderr, "\033[1;35m[March] 原地踏步 5 秒...\033[0m\n");
    motion.stand();
    rclcpp::sleep_for(std::chrono::seconds(1));
    // 真机: motion_servo_cmd 303 + vel_des=[0,0,0] @20Hz；仿真: gamepad
    for (int i = 0; i < 100; i++) {
        motion.set_walk_velocity(0.0f, 0.0f, 0.0f);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    motion.stop();
    fprintf(stderr, "\033[1;32m[March] 原地踏步完成\033[0m\n");
}

// ── 相对转向：从当前 abs_yaw 转 90°（⚠ 反馈用 global_to_robot.rpy[2]，IMU yaw 真机一直 0） ──
void turn_angle_test(MotionCtrl& motion, SensorData& sensor) {
    const float TARGET_DEG = 90.0f;
    const float TURN_SPEED = 0.6f;   // rad/s，+0.6=左转（2026-08-07 已验证方向）
    motion.stand();
    rclcpp::sleep_for(std::chrono::seconds(3));
    float start_yaw = sensor.abs_yaw;   // 地图绝对朝向（SLAM 有数据）
    float target    = start_yaw + TARGET_DEG * M_PI / 180.0f;
    int timeout = 1000;
    fprintf(stderr, "\033[1;35m[Turn] 左转 %.0f° (起始 %.1f°)...\033[0m\n",
            TARGET_DEG, start_yaw * 180.0f / M_PI);
    while (timeout-- > 0) {
        float err = target - sensor.abs_yaw;
        while (err >  M_PI) err -= 2 * M_PI;
        while (err < -M_PI) err += 2 * M_PI;
        if (std::fabs(err) < 0.05f) break;
        motion.set_walk_velocity(0.0f, 0.0f, TURN_SPEED);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stop();
    // ⚠ 跨 ±180° 时相减会错（如 129.8°→219.8° 显示 -140.2°，相减=-272），需环绕归一化
    float d = (sensor.abs_yaw - start_yaw) * 180.0f / M_PI;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    fprintf(stderr, "\033[1;32m[Turn] 实际转 %.1f°\033[0m\n", d);
}

// ── 绝对转向：转到地图坐标系固定角度（SLAM 原点, abs_yaw 闭环） ──
void abs_turn_test(MotionCtrl& motion, SensorData& sensor) {
    const float TARGET_DEG = 90.0f;   // 地图绝对朝向 90°
    motion.stand();
    rclcpp::sleep_for(std::chrono::seconds(3));
    float target = TARGET_DEG * M_PI / 180.0f;
    int timeout = 1000;
    fprintf(stderr, "\033[1;35m[AbsTurn] 转到地图绝对 %.0f° (当前 abs_yaw=%.1f°)...\033[0m\n",
            TARGET_DEG, sensor.abs_yaw * 180.0f / M_PI);
    while (timeout-- > 0) {
        float err = target - sensor.abs_yaw;
        while (err >  M_PI) err -= 2 * M_PI;
        while (err < -M_PI) err += 2 * M_PI;
        if (std::fabs(err) < 0.05f) break;
        motion.set_walk_velocity(0.0f, 0.0f, err > 0 ? 0.6f : -0.6f);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stop();
    fprintf(stderr, "\033[1;32m[AbsTurn] 结束 abs_yaw=%.1f° (目标 %.0f°)\033[0m\n",
            sensor.abs_yaw * 180.0f / M_PI, TARGET_DEG);
}

// ── 前跳 30cm（真机: MotionResultCmd 133；仿真: 旧 LCM JUMP_3D） ──
void jump30_test(MotionCtrl& motion, SensorData& sensor) {
    (void)sensor;
    fprintf(stderr, "\033[1;35m[Jump] 前跳 30cm...\033[0m\n");
    motion.stand();                          // 真机: MotionResultCmd 111 官方站立
    rclcpp::sleep_for(std::chrono::seconds(3));   // 等真正站起（服务异步）
    motion.jump_forward(0.3f);
    rclcpp::sleep_for(std::chrono::seconds(4));   // 等起跳+落地
    motion.stand();
    fprintf(stderr, "\033[1;32m[Jump] 完成\033[0m\n");
}

// ── 前进 N 米（odom 闭环：走满目标距离才停，比时间控制精确） ──
void forward_test(MotionCtrl& motion, SensorData& sensor) {
    const float TARGET_DIST = 0.5f;   // 前进 0.5 米
    const float SPEED       = 0.3f;   // 速度 0.3 m/s
    motion.stand();
    rclcpp::sleep_for(std::chrono::seconds(1));

    float sx = sensor.odom_x, sy = sensor.odom_y;
    float traveled = 0.0f;
    int timeout = 1000;   // 20s 超时保护（防 odom 失效死循环）
    fprintf(stderr, "\033[1;35m[Fwd] 前进 %.1f m @ %.2f m/s (odom闭环)...\033[0m\n",
            TARGET_DIST, SPEED);
    while (traveled < TARGET_DIST && timeout-- > 0) {
        motion.set_walk_velocity(SPEED, 0.0f, 0.0f);   // 前进
        rclcpp::sleep_for(std::chrono::milliseconds(20)); // 50Hz
        float dx = sensor.odom_x - sx, dy = sensor.odom_y - sy;
        traveled = std::sqrt(dx*dx + dy*dy);
    }
    motion.stop();
    fprintf(stderr, "\033[1;32m[Fwd] 实际前进 %.3f m (目标 %.1f)\033[0m\n",
            traveled, TARGET_DIST);
}

// ── RGB 实时预览（DEBUG_VISION 弹窗，按 ESC 退出） ──
void rgb_view_test(MotionCtrl& motion, SensorData& sensor) {
    (void)motion; (void)sensor;
    fprintf(stderr, "\033[1;33m[RGBView] cv::imshow 弹窗中，按 ESC 退出...\033[0m\n");
    // 主循环由 control_loop 100Hz 驱动，on_rgb 自动 imshow
    // 此处仅保持运行直到用户 Ctrl+C
    while (rclcpp::ok()) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

} // namespace behavior

// ── 通用链路测试 ──
namespace behavior_test {

void ping() { fprintf(stderr, "\033[1;36m[Test] pong\033[0m\n"); }
void run_all() {
    fprintf(stderr, "\033[1;36m[Test] run_all ---\033[0m\n");
    ping();
    fprintf(stderr, "\033[1;36m[Test] run_all done\033[0m\n");
}

} // namespace behavior_test
