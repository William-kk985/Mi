#pragma once
#include <vector>

// ═══ LidarLocator — 雷达定位（vision/real，样子实现） ═══
// 预留：Stage6 出口定位（找空旷方向推球出出口）
// 当前：最小可用实现——在激光扫描里找"最大空旷扇区"作为出口方向。
// TODO: 完善算法（距离突变聚类 / 融合里程计 / 与相机找球配合），供 Stage6 使用。

struct LidarExitResult {
    bool  found{false};      // 是否找到出口
    float exit_angle{0.0f};  // 出口方向角（相对雷达 0°，弧度）
    float exit_dist{0.0f};   // 出口方向空旷距离 (m)
};

class LidarLocator {
public:
    // ranges: 各角度距离 (m)；angle_min/angle_inc: 起始角与角增量 (rad)
    // 最小实现：找连续距离 > 2m 的最长扇区，其中心角 = 出口方向
    LidarExitResult find_exit(const std::vector<float>& ranges,
                              float angle_min, float angle_inc);
};
