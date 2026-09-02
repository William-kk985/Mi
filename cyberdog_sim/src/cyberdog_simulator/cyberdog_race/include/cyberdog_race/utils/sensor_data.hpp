#pragma once

#include <vector>
#include <cstdint>

// 传感器数据共享结构，由主循环填充并由各赛段只读访问
struct SensorData {
    // 帧同步与有效标志（等新视觉帧/传感器就绪）
    std::uint64_t vision_seq{0};
    bool  rgb_valid{false};

    // 视觉结果（视觉线程写，主线程读）
    float lane_offset{0.0f};    // 黄线中心偏差，正=偏右
    float lane_curvature{0.0f}; // 弯曲程度（斜率标准差）
    float lane_yaw{0.0f};       // 单线方向角（rad，相对竖直，正值=线向右偏）
    float lane_line_x{0.0f};    // 单线近端 x（归一化[-1,1]，仅单线有效）
    float cmd_yaw{0.0f};        // 当前转向指令（rad，正=左转，供 Web 箭头显示）
    bool  lane_valid{false};    // 黄线是否有效
    bool  lane_both_sides{false}; // 是否双边检测

    float ball_x{0.0f};        // 球在图像中的x坐标（归一化）
    float ball_dist{0.0f};     // 球距离（米）
    bool  ball_found{false};   // 是否找到目标球

    float blue_ball_x{0.0f};   // 蓝球x坐标（归一化）
    float blue_ball_dist{0.0f};
    bool  blue_ball_found{false};

    float white_ball_x{0.0f};    // 白球x坐标（归一化）
    float white_ball_dist{0.0f};
    bool  white_ball_found{false};

    // Stage4 多目标检测（橙球仍用上面 ball_* 的检测）
    bool  football_found{false};
    float football_x{0.0f};
    float football_dist{0.0f};

    bool  limbar_found{false};
    float limbar_x{0.0f};
    float limbar_dist{0.0f};

    bool  coke_found{false};
    float coke_x{0.0f};
    float coke_dist{0.0f};

    bool  obstacle_found{false};
    float obstacle_x{0.0f};
    float obstacle_dist{0.0f};

    // Stage4 区域分隔黄线（实/虚判断，用于借道绕行决策）
    bool  divider_found{false};
    float divider_x{0.0f};
    float divider_dist{0.0f};
    bool  divider_is_dashed{false};

    // IMU
    float yaw{0.0f};           // 当前偏航角（rad）
    float pitch{0.0f};
    float roll{0.0f};

    // 里程计
    float odom_x{0.0f};
    float odom_y{0.0f};
    float body_height{0.25f};  // 身体离地高度（m）
    float abs_yaw{0.0f};       // 地图坐标系绝对朝向（global_to_robot.rpy[2], SLAM固定坐标系）
    float yaw_odom{0.0f};      // odom_out 四元数航向（RT 板腿里程计融合 IMU，46.6Hz，航向锁反馈源）
    bool  odom_yaw_ready{false}; // 是否收到过 odom_out 首包（赛段等就绪用）
    float odom_pos_x{0.0f};    // odom_out 腿里程计位置 x（同源建系点位导航用）
    float odom_pos_y{0.0f};    // odom_out 腿里程计位置 y
    float yaw_imu{0.0f};        // external_imu 四元数航向（500Hz，石径回正抗抖源）
    bool  imu_yaw_ready{false}; // 收到过 external_imu 首包
    float abs_yaw_lp{0.0f};     // abs_yaw 一阶低通（石径 VIO 抖动滤波）
    bool  abs_yaw_lp_init{false};
    float pitch_map{0.0f};     // 地图坐标系俯仰（global_to_robot.rpy[1]，抬头验证用）
    float roll_map{0.0f};      // 地图坐标系横滚/侧倾（global_to_robot.rpy[0]，身躯倾斜验证用）

    // Lidar：前方最近障碍距离
    float lidar_front{10.0f};

    // TOF 四腿离地间隙（head/rear 回调写入）
    // 四个 TOF 最低点 (m)，有效范围 0.15-0.66，用于 Stage5 独木桥检测
    float tof_clearance{0.66f};     // TOF 最低点（0.15~0.66m，0.66=未更新）——脚着地基线
    bool  tof_available{false};      // TOF 是否有可用数据（data_available=true）
    bool  tof_msg_received{false};   // 是否收到过任何 TOF 消息（区分订阅断 vs 运动被抑制）
    float tof_elev_max{0.0f};        // 4 腿 TOF 最大高程——脚抬到最高时的读数（测步高用）
                                     // ⚠ TOF 在四条腿上测抬升高度：脚抬起→读数变大→max=抬腿峰值

    // 超声测距
    float ultrasonic_range{0.0f};  // 0 = 无效/未收到
};
