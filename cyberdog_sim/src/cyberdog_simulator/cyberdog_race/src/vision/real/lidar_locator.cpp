#include "cyberdog_race/vision/real/lidar_locator.hpp"

#include <algorithm>

// 最小实现：找"空旷距离 > 2m"的最长连续扇区，取中心角为出口方向。
// 适合"四周是桥体、某方向是缺口"的 Stage6 场景；后续按需求完善。
LidarExitResult LidarLocator::find_exit(const std::vector<float>& ranges,
                                        float angle_min, float angle_inc) {
    LidarExitResult result;
    if (ranges.empty()) return result;

    const float kOpenDist = 2.0f;
    const int N = static_cast<int>(ranges.size());
    int best_start = -1, best_len = 0;
    int cur_start = -1, cur_len = 0;

    for (int i = 0; i < N; ++i) {
        const bool open = (ranges[i] > kOpenDist);
        if (open) {
            if (cur_start < 0) cur_start = i;
            ++cur_len;
        } else {
            if (cur_len > best_len) { best_len = cur_len; best_start = cur_start; }
            cur_start = -1;
            cur_len = 0;
        }
    }
    if (cur_len > best_len) { best_len = cur_len; best_start = cur_start; }
    if (best_start < 0 || best_len < 3) return result;

    result.found = true;
    result.exit_angle = angle_min + (best_start + best_len / 2.0f) * angle_inc;

    float max_d = 0.0f;
    for (int i = best_start; i < best_start + best_len; ++i)
        max_d = std::max(max_d, ranges[i]);
    result.exit_dist = max_d;
    return result;
}
