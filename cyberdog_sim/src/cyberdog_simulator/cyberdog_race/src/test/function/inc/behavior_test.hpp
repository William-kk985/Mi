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

} // namespace behavior

// ── 通用链路测试（cmake -DUSE_TEST_BEHAVIOR=ON 编译） ──
namespace behavior_test {

void ping();
void run_all();

} // namespace behavior_test
