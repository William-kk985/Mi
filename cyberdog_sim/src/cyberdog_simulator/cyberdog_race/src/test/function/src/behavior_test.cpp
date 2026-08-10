#include "behavior_test.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <cmath>
#include <functional>

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
        case 16: step_height_walk_test(motion, sensor); break;
        case 17: pitch_low_fwd_test(motion, sensor);   break;
        case 18: roll_walk_test(motion, sensor);        break;
        case 19: pitch_unlock_test(motion, sensor);     break;
        case 20: height_low_walk_test(motion, sensor);  break;
        case 21: walk_jump_test(motion, sensor);        break;
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
    fprintf(stderr, "\033[1;35m[Pitch] 抬头 15° 3秒...\033[0m\n");
    for (int i = 0; i < 60; i++) {
        motion.set_body_pitch(-0.26f);   // ⚠ 真机约定：负值=抬头（2026-08-08 舵机方向确认）
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    fprintf(stderr, "\033[1;35m[Pitch] 回正 2秒...\033[0m\n");
    for (int i = 0; i < 40; i++) {
        motion.set_body_pitch(0.0f);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    fprintf(stderr, "\033[1;35m[Pitch] 低头 15° 3秒...\033[0m\n");
    for (int i = 0; i < 60; i++) {
        motion.set_body_pitch(0.26f);   // ⚠ 真机约定：正值=低头（2026-08-08 舵机方向确认）
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

// ── 步高测试（真机 set_step_height 打包毫米 + 带前进短距离）+ 4腿TOF抬腿峰值 ──
// 真机步高已打通：LCM robot_control_cmd(7671) + 打包毫米编码（0.15→150150），起步前设一次生效。
// test8 是原地；本测试带前进（0.2m/s×1.5s≈0.3m/段）验证走路态步高。
// 指标：tof_elev_max = 4腿TOF抬腿峰值（protocol 注释：tof on each of the four legs, elevation info）。
void step_height_walk_test(MotionCtrl& motion, SensorData& sensor) {
    const float SPEED = 0.2f;   // 慢走 0.2 m/s（1.5s ≈ 0.3m/段）
    motion.stand();
    rclcpp::sleep_for(std::chrono::seconds(2));   // 等站稳

    // ── 0) 静止对照：TOF 应正常 ──
    {
        float t_min = 99.0f; int n_avail = 0;
        fprintf(stderr, "\033[1;36m[StepH] 0) 静止2s — TOF应正常\033[0m\n");
        for (int i = 0; i < 20; i++) {
            if (i % 5 == 0) {
                float t = sensor.tof_clearance;
                bool  a = sensor.tof_available;
                if (a) { n_avail++; if (t < t_min) t_min = t; }
                fprintf(stderr, "    t=%.2fs tof=%.3f%s%s\n", i * 0.05f, t,
                        a ? "" : "(无数据)", sensor.tof_msg_received ? "" : " [从未收到TOF消息!]");
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        fprintf(stderr, "\033[1;32m[StepH] 0) 静止: TOF min=%.3f 可用%d/4, msg_received=%d\033[0m\n",
                t_min, n_avail, (int)sensor.tof_msg_received);
    }

    auto walk = [&](const char* tag, float h) {
        sensor.tof_elev_max = 0.0f;   // 清零抬腿峰值追踪（本段）
        float sx = sensor.odom_x, sy = sensor.odom_y;
        fprintf(stderr, "\033[1;36m[StepH] %s 步高=%.2fm 慢走1.5s\033[0m\n", tag, h);
        if (h >= 0.0f)
            motion.set_step_height(h, h);   // 正式接口(打包毫米)：起步前设一次，中途不改
        for (int i = 0; i < 75; i++) {
            motion.set_walk_velocity(SPEED, 0.0f, 0.0f);   // 慢走前进
            if (i % 10 == 0)
                fprintf(stderr, "    t=%.2fs tof=%.3f elev_max=%.3f body_h=%.3f\n",
                        i * 0.02f, sensor.tof_clearance, sensor.tof_elev_max, sensor.body_height);
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }
        motion.stop();
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        float d = std::sqrt((sensor.odom_x-sx)*(sensor.odom_x-sx) + (sensor.odom_y-sy)*(sensor.odom_y-sy));
        fprintf(stderr, "\033[1;32m[StepH] %s: 走%.3fm elev_max=%.3f (msg=%d)\033[0m\n",
                tag, d, sensor.tof_elev_max, (int)sensor.tof_msg_received);
    };

    // 基准 0.15 + 附近标定：正式 set_step_height(打包毫米) 起步前设一次，带前进短距离
    walk("A)参考(不设)", -1.0f);
    walk("B)0.10m", 0.10f);
    walk("C)0.15m(基准)", 0.15f);
    walk("D)0.20m", 0.20f);
    fprintf(stderr, "\033[1;32m[StepH] 完成\033[0m\n");
}

// ── 低头前进（身躯姿态变化 + 前进组合，test17） ──
// 交替(201+303)与 des_roll_pitch_height 参数均失败：走路时 pitch 被速度控制器冲回 0。
// 【2026-08-08 源码确认】仿真/真机同源 convex_mpc_loco_gaits.cpp:2305：
//   rpy_cmd_[1] = ctrl_cmd_->rpy_des[1];   ← 走路时 pitch 目标直接读 303 命令的 rpy_des[1]!
//   rpy_des_(1) = WrapRange(..., scale*min, scale*max)  ← TROT 默认 ±0.1rad(±5.7°)，限位随速度负放大(x_effect_scale=-0.55)
//   safety_checker: locomotion+大pitch 专门放行（只禁 lift error）
// → 正确姿势：303 WALK 命令里直接带 rpy_des[1]=pitch（set_walk_velocity_pitch），20Hz 持续。
// ⚠ 2026-08-08 三测：方向按真机约定【正值=低头】PITCH=+0.30（上坡桥面需要低头），0.15慢速 + 走 0.5m
//   步态夹持 ±5.7°，另加 B2 段试 LCM 7668 参数 des_roll_pitch_height[1] 能否破限
// 反馈用 pitch_map(global_to_robot.rpy[1])：+0.30 会被步态夹到 ~+5.7° 起。
void pitch_low_fwd_test(MotionCtrl& motion, SensorData& sensor) {
    const float PITCH = 0.30f;    // 低头 0.30 rad（真机约定正值=低头，2026-08-08 舵机方向确认）
    const float SPEED = 0.15f;    // 慢速 0.15 m/s（pitch 限位随速度负放大，越慢范围越大）
    motion.stand();
    rclcpp::sleep_for(std::chrono::seconds(2));
    fprintf(stderr, "\033[1;35m[PitchFwd] 起点 pitch_map=%.1f°\033[0m\n",
            sensor.pitch_map * 180.0f / M_PI);

    // ── A) 单独低头 2s：验证 201 低头本身正常（+0.30=低头） ──
    fprintf(stderr, "\033[1;36m[PitchFwd] A) set_body_pitch(+0.30) 2s 低头\033[0m\n");
    for (int i = 0; i < 100; i++) {
        motion.set_body_pitch(PITCH);
        if (i % 25 == 0)
            fprintf(stderr, "    t=%.1fs pitch_map=%.1f°\n", i * 0.02f, sensor.pitch_map * 180.0f / M_PI);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stop();

    // ── B) 303 WALK 前进走满 0.5m（odom闭环），对比两种 pitch 通道 ──
    const float TARGET_DIST = 0.5f;

    // 公共走段 lambda：按给定发命令回调走满 TARGET_DIST
    auto walk_phase = [&](const char* tag, const std::function<void()>& send_cmd) {
        float sx = sensor.odom_x, sy = sensor.odom_y;
        float traveled = 0.0f;
        int timeout = 1500;   // 30s 超时保护（慢速）
        fprintf(stderr, "\033[1;36m[PitchFwd] %s 走满 %.1fm\033[0m\n", tag, TARGET_DIST);
        while (traveled < TARGET_DIST && timeout-- > 0) {
            send_cmd();
            if (timeout % 10 == 0)
                fprintf(stderr, "    pitch_map=%.1f° 走%.3fm\n",
                        sensor.pitch_map * 180.0f / M_PI, traveled);
            rclcpp::sleep_for(std::chrono::milliseconds(20));
            float dx = sensor.odom_x - sx, dy = sensor.odom_y - sy;
            traveled = std::sqrt(dx*dx + dy*dy);
        }
        motion.stop();
        fprintf(stderr, "\033[1;32m[PitchFwd] %s 走满 %.3fm, pitch_map=%.1f° → %s\033[0m\n",
                tag, traveled, sensor.pitch_map * 180.0f / M_PI,
                sensor.pitch_map > 0.05f ? "低头保持!" : "低头无效");
        return sensor.pitch_map;
    };

    // B1) 纯 303 命令低头（现状，预期 +5.7° 夹持）
    walk_phase("B1) 303带rpy_des[1]=0.30 前进",
               [&]{ motion.set_walk_velocity_pitch(SPEED, 0, 0, PITCH); });

    // B2) LCM 7668 参数 des_roll_pitch_height[1]=0.30（roll 走参数通道能破限28°，试 pitch [1]）
    //    仿真源码说走路时 pitch 不读参数，但真机 roll 已证明与仿真不同，值得一试
    motion.set_body_params_lcm(0.0f, PITCH, 0.25f);   // 参数 pitch=0.30（真机走路时若读则无夹持）
    float p2 = walk_phase("B2) LCM参数pitch=0.30 + 303前进(命令也带0.30)",
               [&]{ motion.set_walk_velocity_pitch(SPEED, 0, 0, PITCH); });
    fprintf(stderr, "\033[1;33m[PitchFwd] B2 pitch_map=%.1f° （>5.7° 说明参数通道破限！）\033[0m\n",
            p2 * 180.0f / M_PI);

    // ── C) LCM/303 回正 + 停 ──
    fprintf(stderr, "\033[1;36m[PitchFwd] C) 回正 1s\033[0m\n");
    motion.set_body_params_lcm(0.0f, 0.0f, 0.25f);   // 参数回正
    for (int i = 0; i < 50; i++) {
        motion.set_walk_velocity_pitch(0.0f, 0, 0, 0.0f);   // 原地踏步回正
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stop();
    fprintf(stderr, "\033[1;32m[PitchFwd] 完成, pitch_map=%.1f°\033[0m\n",
            sensor.pitch_map * 180.0f / M_PI);
}

// ── roll 走路保持侧倾（身躯姿态变化 + 前进组合，test18） ──
// 【2026-08-08 排查 → LCM 参数通道打通方案】
// - ❌ des_roll_pitch_height YamlParam(ROS topic)：真机无节点订阅 yaml_parameter，死通道
// - ❌ 303+rpy_des[0]：走路时 roll 命令通道关闭（运控只启用 pitch），冲回0
// - ❌ 201+rpy_des[0]+vel_des：201 忽略 vel_des，姿态能保持但不走
// - ✅ 【本方案】LCM interface_request 通道（control_parameter_request_lcmt, kSET_USER_PARAM_BY_NAME）
//   直接改 RT 板运控 user_params des_roll_pitch_height[0] → 走路时 roll 保持（convex_mpc_loco_gaits.cpp:2304
//   走路时 rpy_des_(0)=user_params_->des_roll_pitch_height[0]，正是这个参数！）
// 反馈用 roll_map(global_to_robot.rpy[0])。
void roll_walk_test(MotionCtrl& motion, SensorData& sensor) {
    const float ROLL = 0.52f;    // 侧倾 0.52 rad ≈ 30°
    const float SPEED = 0.2f;    // 前进 0.2 m/s
    const float TARGET_DIST = 0.3f;
    motion.stand();
    rclcpp::sleep_for(std::chrono::seconds(2));
    fprintf(stderr, "\033[1;35m[RollWalk] 起点 roll_map=%.1f°\033[0m\n",
            sensor.roll_map * 180.0f / M_PI);

    // ── A) 单独 201 roll 2s：验证 roll 本身正常（30°） ──
    fprintf(stderr, "\033[1;36m[RollWalk] A) set_body_roll(+0.52) 2s 侧倾30°\033[0m\n");
    for (int i = 0; i < 100; i++) {
        motion.set_body_roll(ROLL);
        if (i % 25 == 0)
            fprintf(stderr, "    t=%.1fs roll_map=%.1f°\n", i * 0.02f, sensor.roll_map * 180.0f / M_PI);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stop();

    // ── B) LCM interface_request 设 des_roll_pitch_height=[roll,0,0.25] + 303 前进 ──
    // RT 板运控 user_params 被直接改写 → 走路时 roll 目标 = 0.52（convex_mpc 直读该参数）
    float sx = sensor.odom_x, sy = sensor.odom_y;
    float traveled = 0.0f;
    int timeout = 1000;   // 20s 超时保护
    fprintf(stderr, "\033[1;36m[RollWalk] B) LCM参数 roll=0.52 + 303前进 走满 %.1fm\033[0m\n", TARGET_DIST);
    motion.set_body_params_lcm(ROLL, 0.0f, 0.25f);   // LCM 直改 RT 板用户参数（走路时保持）
    while (traveled < TARGET_DIST && timeout-- > 0) {
        motion.set_walk_velocity(SPEED, 0, 0);   // 303 前进（roll 靠 LCM 参数保持）
        if (timeout % 10 == 0)
            fprintf(stderr, "    roll_map=%.1f° 走%.3fm\n",
                    sensor.roll_map * 180.0f / M_PI, traveled);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
        float dx = sensor.odom_x - sx, dy = sensor.odom_y - sy;
        traveled = std::sqrt(dx*dx + dy*dy);
    }
    motion.stop();
    fprintf(stderr, "\033[1;32m[RollWalk] B) 走满 %.3fm, roll_map=%.1f° → %s\033[0m\n",
            traveled, sensor.roll_map * 180.0f / M_PI,
            sensor.roll_map > 0.10f ? "LCM参数侧倾保持!" : "LCM参数未生效");

    // ── C) LCM 参数回正 + 停 ──
    fprintf(stderr, "\033[1;36m[RollWalk] C) LCM参数回正 roll=0 pitch=0 h=0.25 1s\033[0m\n");
    motion.set_body_params_lcm(0.0f, 0.0f, 0.25f);
    for (int i = 0; i < 50; i++) {
        motion.set_body_roll(0.0f);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stop();
    fprintf(stderr, "\033[1;32m[RollWalk] 完成, roll_map=%.1f°\033[0m\n",
            sensor.roll_map * 180.0f / M_PI);
}

// ── pitch 破限（走路大角度低头，test19，自己研制方案） ──
// 【原理】走路 pitch 被 gait WrapRange(±0.1rad) 夹死，但夹持系数 rpy_cmd_scale_(1) 每周期乘
//   (fabs(vx) * user_params_->x_effect_scale_pos + 1.0)（convex_mpc_loco_gaits.cpp:2420）。
//   x_effect_scale_pos 是用户参数（默认 -0.55，负=走路时缩小 pitch 范围）。
//   → 通过 LCM 7668 interface_request（已打通）把它改成大正数（+30）→ 走路时 pitch 限位被放大
//     7~11 倍 → ±0.25 命令穿透 WrapRange，像 roll 参数通道一样无夹持！
// 注意：该参数同时放大 vel_cmd_scale_(1)/(2)（y/yaw 速度限），前进直行无影响，测完必须复原。
void pitch_unlock_test(MotionCtrl& motion, SensorData& sensor) {
    const float PITCH = 0.25f;    // 低头 0.25 rad（真机约定正值=低头）
    const float SPEED = 0.2f;     // 前进 0.2 m/s（触发 x_effect_scale_pos 放大）
    const float TARGET_DIST = 0.5f;
    const double SCALE_HACK = 30.0;   // x_effect_scale_pos 放大值
    const double SCALE_RESTORE = -0.55;  // 默认值（cyberdog2-ctrl-user-parameters.yaml）
    motion.stand();
    rclcpp::sleep_for(std::chrono::seconds(2));
    fprintf(stderr, "\033[1;35m[PitchUnlock] 起点 pitch_map=%.1f°\033[0m\n",
            sensor.pitch_map * 180.0f / M_PI);

    // ── A) 201 低头 +0.25 2s：参考（原地无夹持，预期 ~+14°） ──
    fprintf(stderr, "\033[1;36m[PitchUnlock] A) set_body_pitch(+0.25) 2s 低头(参考)\033[0m\n");
    for (int i = 0; i < 100; i++) {
        motion.set_body_pitch(PITCH);
        if (i % 25 == 0)
            fprintf(stderr, "    t=%.1fs pitch_map=%.1f°\n", i * 0.02f, sensor.pitch_map * 180.0f / M_PI);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stop();

    // ── B) LCM 设 x_effect_scale_pos=+30 + 303 前进带低头，走满 0.5m ──
    motion.set_user_param_double_lcm("x_effect_scale_pos", SCALE_HACK);   // 破限开关
    float sx = sensor.odom_x, sy = sensor.odom_y;
    float traveled = 0.0f;
    int timeout = 1500;   // 30s 超时保护
    fprintf(stderr, "\033[1;36m[PitchUnlock] B) x_effect_scale_pos=+30 + 303低头0.25 走满 %.1fm\033[0m\n", TARGET_DIST);
    while (traveled < TARGET_DIST && timeout-- > 0) {
        motion.set_walk_velocity_pitch(SPEED, 0, 0, PITCH);   // 303 前进 + 带低头
        if (timeout % 10 == 0)
            fprintf(stderr, "    pitch_map=%.1f° 走%.3fm\n",
                    sensor.pitch_map * 180.0f / M_PI, traveled);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
        float dx = sensor.odom_x - sx, dy = sensor.odom_y - sy;
        traveled = std::sqrt(dx*dx + dy*dy);
    }
    motion.stop();
    fprintf(stderr, "\033[1;32m[PitchUnlock] B) 走满 %.3fm, pitch_map=%.1f° → %s\033[0m\n",
            traveled, sensor.pitch_map * 180.0f / M_PI,
            sensor.pitch_map > 0.12f ? "破限成功!大角度低头保持" : "仍未破限(±5.7°)");

    // ── C) 复原 x_effect_scale_pos + 回正 ──
    fprintf(stderr, "\033[1;36m[PitchUnlock] C) 复原 x_effect_scale_pos=-0.55 回正\033[0m\n");
    motion.set_user_param_double_lcm("x_effect_scale_pos", SCALE_RESTORE);
    for (int i = 0; i < 50; i++) {
        motion.set_walk_velocity_pitch(0.0f, 0, 0, 0.0f);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stop();
    fprintf(stderr, "\033[1;32m[PitchUnlock] 完成, pitch_map=%.1f°\033[0m\n",
            sensor.pitch_map * 180.0f / M_PI);
}

// ── 降低身体高度（test20） ──
// 【2026-08-10 路线26】kWalkWave(60) + motion 参数降身高 — 唯一"走+降"都通的链路
// 路线22: motion 参数降身高 ✅ (0.235→0.143) 不崩，但 kTrotInOut(56) 不走
// 路线4: kWalkWave(60) ✅ 能走（MotionGaits 里唯一能前进的步态）
// → 站定写 motion[2]=0.16 → kWalkWave(60) 走10秒（SetWalkWaveParams 读 motion[2]）
void height_low_walk_test(MotionCtrl& motion, SensorData& sensor) {
    const float SPEED = 0.2f;

    // 站定写 motion 参数（安全不崩，路线22验证）
    auto stand_set_motion = [&](float h) {
        motion.stand();
        rclcpp::sleep_for(std::chrono::seconds(3));
        motion.set_body_params_motion_lcm(0, 0, h);
        rclcpp::sleep_for(std::chrono::seconds(1));
        fprintf(stderr, "    站定写motion %.2f body_height=%.3f\n", h, sensor.body_height);
    };

    motion.stand();
    rclcpp::sleep_for(std::chrono::seconds(3));
    fprintf(stderr, "\033[1;35m[HeightLow] 起点 body_height=%.3f\033[0m\n", sensor.body_height);

    // 1) 站定写 motion[2]=0.16（WalkWave 身高 ≈ motion+0.02≈0.18，不擦地）
    fprintf(stderr, "\033[1;36m[HeightLow] 1) 站定写motion 0.16\033[0m\n");
    stand_set_motion(0.16f);

    // 2) kWalkWave(60) 走路 10秒
    fprintf(stderr, "\033[1;36m[HeightLow] 2) kWalkWave(60) 走10秒\033[0m\n");
    float sx = sensor.odom_x, sy = sensor.odom_y;
    float min_h = sensor.body_height, max_h = sensor.body_height;
    for (int i = 0; i < 500; i++) {
        motion.set_walk_wave(SPEED, 0, 0);
        if (i % 50 == 0) {
            float tr = std::sqrt((sensor.odom_x-sx)*(sensor.odom_x-sx) +
                                 (sensor.odom_y-sy)*(sensor.odom_y-sy));
            fprintf(stderr, "    t=%2.1fs body_height=%.3f 走%.3fm\n",
                    i * 0.02f, sensor.body_height, tr);
        }
        if (sensor.body_height < min_h) min_h = sensor.body_height;
        if (sensor.body_height > max_h) max_h = sensor.body_height;
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stop();
    float traveled = std::sqrt((sensor.odom_x-sx)*(sensor.odom_x-sx) +
                               (sensor.odom_y-sy)*(sensor.odom_y-sy));
    bool low = min_h < 0.18f;
    bool walk = traveled > 0.3f;  // WalkWave 步态慢，放宽判据
    fprintf(stderr, "\033[1;32m[HeightLow] 走%.3fm 身高 min=%.3f max=%.3f → %s\033[0m\n",
            traveled, min_h, max_h,
            (low && walk) ? "✓✓ WalkWave低姿走路成功!!!" :
            (low ? "✓ 低姿到位但走不动" : (walk ? "✗ 能走但身高未降" : "✗ 走不动+身高未降")));

    // 3) 复位
    fprintf(stderr, "\033[1;36m[HeightLow] 3) 站定写回motion 0.225 + 201\033[0m\n");
    stand_set_motion(0.225f);
    for (int i = 0; i < 100; i++) {
        motion.set_body_pose_height(0.235f);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stop();
    fprintf(stderr, "\033[1;32m[HeightLow] 完成 body_height=%.3f\033[0m\n", sensor.body_height);
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

// ── 前进0.3m + 跳跃（test21） ──
void walk_jump_test(MotionCtrl& motion, SensorData& sensor) {
    const float TARGET = 0.3f;
    const float SPEED  = 0.25f;

    fprintf(stderr, "\033[1;35m[WalkJump] 前进 %.1fm → 跳跃\033[0m\n", TARGET);
    motion.stand();
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 1) 前进 0.3m（odom 闭环）
    float sx = sensor.odom_x, sy = sensor.odom_y;
    float traveled = 0;
    int timeout = 500;  // 10s 超时
    while (traveled < TARGET && timeout-- > 0) {
        motion.set_walk_velocity_step(SPEED, 0, 0, 0.15f);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
        float dx = sensor.odom_x - sx, dy = sensor.odom_y - sy;
        traveled = std::sqrt(dx*dx + dy*dy);
        if (timeout % 25 == 0)
            fprintf(stderr, "    走%.3fm/%.1f\n", traveled, TARGET);
    }
    motion.stop();
    fprintf(stderr, "\033[1;32m[WalkJump] 前进 %.3fm\033[0m\n", traveled);

    // 2) 跳跃
    fprintf(stderr, "\033[1;33m[WalkJump] 跳!\033[0m\n");
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    motion.jump_forward(0.3f);
    rclcpp::sleep_for(std::chrono::seconds(2));

    motion.stand();
    fprintf(stderr, "\033[1;32m[WalkJump] 完成\033[0m\n");
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
