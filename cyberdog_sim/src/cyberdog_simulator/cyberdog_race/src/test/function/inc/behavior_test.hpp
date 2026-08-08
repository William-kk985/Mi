#pragma once
#include "cyberdog_race/motion_ctrl.hpp"
#include "cyberdog_race/sensor_data.hpp"
#include <cstdio>

// ── 赛段运动行为测试（DEBUG_TEST_BEHAVIOR 触发） ──
namespace behavior {

void run_test(MotionCtrl& motion, SensorData& sensor, int test_id);

void jump_test(MotionCtrl& motion, SensorData& sensor);          // 1: 跳跃
void scan_ball_test(MotionCtrl& motion, SensorData& sensor);     // 2: 扫描找球
void crouch_test(MotionCtrl& motion, SensorData& sensor);        // 3: 蹲下
void navigate_test(MotionCtrl& motion, SensorData& sensor);      // 4: 路径点导航
void turn_test(MotionCtrl& motion, SensorData& sensor);          // 5: 原地转向
void stand_lie_test(MotionCtrl& motion, SensorData& sensor);     // 6: 起立趴下
void pitch_test(MotionCtrl& motion, SensorData& sensor);         // 7: 俯仰角
void step_height_test(MotionCtrl& motion, SensorData& sensor);   // 8: 步高
void sensor_check_test(MotionCtrl& motion, SensorData& sensor);   // 9: 传感器链路检查
void rgb_view_test(MotionCtrl& motion, SensorData& sensor);       // 10: RGB实时预览(cv::imshow)
void march_in_place_test(MotionCtrl& motion, SensorData& sensor); // 11: 原地踏步(servo 303 vel=0)
void forward_test(MotionCtrl& motion, SensorData& sensor);         // 12: 前进N米(odom闭环)
void jump30_test(MotionCtrl& motion, SensorData& sensor);          // 13: 前跳30cm(MotionResultCmd 133)
void turn_angle_test(MotionCtrl& motion, SensorData& sensor);      // 14: 原地转90°(相对, IMU yaw闭环)
void abs_turn_test(MotionCtrl& motion, SensorData& sensor);        // 15: 绝对转向(地图坐标系, abs_yaw闭环)
void step_height_walk_test(MotionCtrl& motion, SensorData& sensor); // 16: 步高切换(303+step_height, 原地踏步观察抬腿高低)
void pitch_low_fwd_test(MotionCtrl& motion, SensorData& sensor);    // 17: 低头前进(303带rpy_des[1]=pitch, pitch_map验证)
void roll_walk_test(MotionCtrl& motion, SensorData& sensor);        // 18: roll走路侧倾(des_roll_pitch_height[0] YamlParam, roll_map验证)
void pitch_unlock_test(MotionCtrl& motion, SensorData& sensor);     // 19: pitch破限(x_effect_scale_pos=+30 放大走路pitch限位)
void segmented_pitch_walk_test(MotionCtrl& motion, SensorData& sensor); // 20: 分段低头前进(201大姿态↔303前进交替, 大pitch+移动)

} // namespace behavior

// ── 通用链路测试（cmake -DUSE_TEST_BEHAVIOR=ON 编译） ──
namespace behavior_test {

void ping();
void run_all();

} // namespace behavior_test
