#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第二赛段：荒野寻珠
// 路径点导航 + 原地扫描 + 橙色球冲击
class Stage2 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    bool is_done() override;

private:
    bool done_{false};

    // 状态机
    enum class State {
        MOVE_TO_POINT,   // 里程计导航到路径点
        TURN_TO_YAW,     // 转向目标朝向
        SCAN_LEFT,       // 左转40度扫描
        SCAN_RIGHT,      // 右转40度扫描
        HIT_BALL,        // 冲向橙色球
        BACK_TO_PATH,    // 退回路径点
        DONE
    } state_{State::MOVE_TO_POINT};

    // 路径点结构
    struct WayPoint {
        float x, y;       // 目标坐标
        float yaw;        // 到达后的朝向
        bool scan;        // 是否在此点扫描
    };

    // 路径点序列
    static constexpr int NUM_WP = 12;
    WayPoint waypoints_[NUM_WP];
    int wp_idx_{0};

    // 当前导航目标
    float target_x_{0}, target_y_{0}, target_yaw_{0};
    float start_x_{0}, start_y_{0};  // 冲击前记录位置，用于退回

    // 扫描状态
    float scan_start_yaw_{0};
    bool  scan_found_{false};
    int   scan_dir_{1};  // 1=先左，-1=先右

    // 到达判断阈值
    static constexpr float POS_THRESH = 0.08f;
    static constexpr float YAW_THRESH = 0.05f;
    static constexpr float BALL_DIST_THRESH = 0.5f;  // 只撞0.5m内的球
    static constexpr float SCAN_ANGLE = 0.7f;  // 扫描角度约40度

    // 运动参数
    static constexpr float MOVE_SPEED  = 0.25f;
    static constexpr float TURN_SPEED  = 0.15f;
    static constexpr float HIT_SPEED   = 0.35f;

    void navigate_to(float tx, float ty);
    void turn_to(float target_yaw);
    bool reached_pos(float tx, float ty);
    bool reached_yaw(float target_yaw);
    bool ball_in_arena();
    void next_waypoint();
};
