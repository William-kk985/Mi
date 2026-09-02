#pragma once
#include "cyberdog_race/stages/stage_base.hpp"

// 第二赛段：荒野寻珠
// 路径点导航 + 原地扫描 + 橙色球冲击
class Stage2 : public StageBase {
public:
    using StageBase::StageBase;
    void init() override;
    void run() override;
    [[nodiscard]] bool is_done() override;

private:
    bool done_{false};

    // 状态机
    enum class State {
        MOVE_TO_POINT,
        TURN_TO_YAW,
        SCAN_LEFT,          // 左转到40°
        SCAN_LEFT_CHECK,    // 停下识别球
        SCAN_LEFT_RETURN,   // 转回y正方向，左扫结束
        SCAN_RIGHT,         // 右转到40°
        SCAN_RIGHT_CHECK,   // 停下识别球
        SCAN_RIGHT_RETURN,  // 转回y正方向，右扫结束
        HIT_BALL,
        DONE
    } state_{State::MOVE_TO_POINT};

    // 路径点结构
    struct WayPoint {
        float x, y;       // 目标坐标
        float yaw;        // 到达后的朝向
        bool scan;        // 是否在此点扫描
    };

    // 路径点序列（实际使用7个，见 init()）
    static constexpr int NUM_WP = 7;
    static_assert(NUM_WP <= 12, "路径点数组超出预期上限，请检查 NUM_WP");
    WayPoint waypoints_[NUM_WP];
    int wp_idx_{0};

    // 当前导航目标
    float target_x_{0}, target_y_{0}, target_yaw_{0};

    // 扫描状态
    float scan_start_yaw_{0};
    bool  scan_found_{false};
    bool  scan_done_{false};
    int   scan_wait_{0};
    int   scan_confirm_{0};  // 转动过程中连续检测到球的帧数
    bool  hit_started_{false};
    float hit_start_x_{0}, hit_start_y_{0};
    bool  exit_started_{false};
    bool  exit_turning_{false};
    float exit_start_x_{0}, exit_start_y_{0};
    float exit_target_yaw_{0};

    // 到达判断阈值
    static constexpr float POS_THRESH = 0.08f;
    static constexpr float YAW_THRESH = 0.05f;
    static constexpr float BALL_DIST_THRESH = 0.80f;
    static constexpr float SCAN_ANGLE       = 0.7f;
    static constexpr float SCAN_WAIT_FRAMES    = 100;  // 停下识别等待帧数

    // 运动参数
    static constexpr float MOVE_SPEED       = 1.0f;
    static constexpr float TURN_SPEED       = 0.7f;  // 路径点转向速度
    static constexpr float SCAN_TURN_SPEED  = 0.25f;  // 扫描时转向速度（慢，方便识别）
    static constexpr float HIT_SPEED        = 0.8f;

    void navigate_to(float tx, float ty);
    void turn_to(float target_yaw);
    [[nodiscard]] bool reached_pos(float tx, float ty);
    [[nodiscard]] bool reached_yaw(float target_yaw);
    void next_waypoint();
};