// ═══════════════════════════════════════════════════════════
// motion_height_test.cpp — 头顶60cm动作测试 (2026-08-16 仅记录, 未接入CMake)
// ═══════════════════════════════════════════════════════════
// 背景: 任务可能需要狗"头顶到达60cm高度"。
// 官方动作全集 (protocol/msg/motion_id.hpp, NX: /opt/ros2/cyberdog/include):
//   101 GETDOWN  111 RECOVERYSTAND  112 WALK_STAND
//   121 BACK_FLIP(后空翻)          123 TWO_LEG_STAND(双腿站立)★
//   124 ROLL_OVER                  126 JUMP_UPSTAIR(跳上台阶)
//   130~135 JUMP3D左右90°/前60/前30/左20/右20cm
//   136 JUMP3D_UP30CM(垂直上跳30cm)★
//   137 JUMP_DOWNSTAIR             201/202 力控  211/212 位控
//   301 JUMP_BOUND  302 JUMP_PRONK  303 WALK_USERTROT
//
// 候选方案:
//   ① 123 TWO_LEG_STAND: 后腿站立前腿抬起, 身体竖直 → 稳态保持, 头顶最高(约50~60cm, 待实测)
//      源码依据: fsm_state_two_leg_stand.cpp 前脚抬到 foot_pos[2]≥-0.25m,
//               H = L1*cos(-0.69)+L2*cos(0.7) 为腿部几何解算
//   ② 136 JUMP3D_UP30CM: 原地垂直跳30cm → 站立头顶(~42cm)+30cm≈72cm峰值, 瞬态
//   ③ 121 BACK_FLIP: 后空翻特技, 翻越中头顶最高, 不适合比赛任务
//
// 调用方式(真机): motion_.send_result_cmd(123) / send_result_cmd(136)
//   MotionCtrl 已有 send_result_cmd(int motion_id) → MotionResultCmd 服务
//   ⚠ 跳跃类需狗处于站立状态且服务就绪(wait_motion_result_ready)
//
// 若要实测, 推荐接入方式: DEBUG_TEST_BEHAVIOR 加一个 TEST_BEHAVIOR 值,
// 在 race_controller 的测试分支里依次执行:
//   站立 → 录一次头顶高度(可用TOF/超声或外部测量) → 123双腿站立 → 录高度
//   → 恢复站立 → 136垂直跳 → 录高度
// 参考下方伪代码 (不参与编译):
// ═══════════════════════════════════════════════════════════

#if 0  // 伪代码记录, 未启用
#include "cyberdog_race/motion_ctrl.hpp"

void test_height(MotionCtrl& motion) {
    // 1. 确保运动服务就绪
    if (!motion.wait_motion_result_ready(5)) return;

    // 2. 站立基准
    motion.send_result_cmd(111);            // RECOVERYSTAND
    rclcpp::sleep_for(std::chrono::seconds(3));
    // ← 此刻用外部分度尺量头顶高度(基准, 约42cm)

    // 3. 双腿站立 (稳态高姿态)
    motion.send_result_cmd(123);            // TWO_LEG_STAND ★
    rclcpp::sleep_for(std::chrono::seconds(5));
    // ← 量头顶高度, 判断是否≥60cm

    // 4. 恢复
    motion.send_result_cmd(111);
    rclcpp::sleep_for(std::chrono::seconds(3));

    // 5. 垂直上跳 (瞬态峰值)
    motion.send_result_cmd(136);            // JUMP3D_UP30CM ★
    rclcpp::sleep_for(std::chrono::seconds(3));
    // ← 录视频逐帧看头顶峰值(站立42cm+30cm≈72cm)
}
#endif

// 结论记录 (2026-08-16):
//   - 机身离地60cm: 不可能 (固件 pos_cmd_max_[2]=站高+0.07=0.32m 硬上限)
//   - 头顶60cm: 首选 123 TWO_LEG_STAND(稳态); 备选 136 JUMP3D_UP30CM(瞬态)
//   - 跳跃类注意: 官方 jump 注释 "jump 20cm box" 为越障设计, 垂直跳136专用
