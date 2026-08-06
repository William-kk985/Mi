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
