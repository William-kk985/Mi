#include "cyberdog_race/behavior_test.hpp"
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
    motion.set_pitch(-0.26f); rclcpp::sleep_for(std::chrono::seconds(1));
    motion.set_pitch(0.0f);   rclcpp::sleep_for(std::chrono::seconds(1));
    motion.set_pitch(0.26f);  rclcpp::sleep_for(std::chrono::seconds(1));
    motion.set_pitch(0.0f);
}

void step_height_test(MotionCtrl& motion, SensorData& sensor) {
    (void)sensor;
    motion.set_step_height(0.35f, 0.35f); rclcpp::sleep_for(std::chrono::seconds(1));
    motion.set_step_height(0.06f, 0.06f); rclcpp::sleep_for(std::chrono::seconds(1));
    motion.set_step_height(0.20f, 0.20f);
}

} // namespace behavior
