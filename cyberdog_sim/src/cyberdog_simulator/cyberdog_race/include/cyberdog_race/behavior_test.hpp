#pragma once
#include "cyberdog_race/motion_ctrl.hpp"
#include "cyberdog_race/sensor_data.hpp"

// 行为测试模块
// 独立测试某个算法或功能，不依赖赛段状态机
// 在 debug_config.hpp 中定义 DEBUG_TEST_BEHAVIOR 并设置 TEST_BEHAVIOR

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

} // namespace behavior
