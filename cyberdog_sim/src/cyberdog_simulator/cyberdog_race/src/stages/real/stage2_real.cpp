#include "cyberdog_race/stages/real/stage2_real.hpp"
#include "cyberdog_race/debug_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ═══════════════════════════════════════════════════════════
// Stage2Real 真机版 — 第2赛段 (2026-08-13 四轮S形)
// 衔接: 前进0.92m → 左转90° (Stage1结束后进入赛道)
// 轮1: 前进2.8m → 右转90° → 左右扫描(左45°停2s/右90°停2s) → 回正左45° → 前进0.75m
// 轮2: 右转90° → 前进2.8m → 左转90° → 左右扫描 → 回正 → 前进0.75m
// 轮3: 左转90° → 前进2.8m → 右转90° → 左右扫描 → 回正 → 前进0.75m
// 轮4: 右转90° → 前进2.8m → 左转90° → 左右扫描 → 结束
// 找球: 每个扫描角度 识别到橙色球(≤0.7m)→前进0.2m再退回; 没有→2秒后转下一个角度
// 方向反馈用 abs_yaw(global_to_robot.rpy[2]), IMU yaw 真机恒0 别用 (README 2026-08-11)
// 前进回正: 死区0.03rad + 增益0.5 (abs_yaw 定位噪声大, 2026-08-12 实测日志验证)
// 转向: 0.45 (实测 yaw=0.6 实际≈2rad/s, 2026-08-12)
// 位置保持: 踏步后退>2cm自动顶回 + 转向后ADJUST修正漂移 (2026-08-13)
// ═══════════════════════════════════════════════════════════

namespace {

// ═══ Stage2 动作参数 (四轮S形, 2026-08-13 重构) ═══
constexpr float WALK_V      = 0.30f;    // 前进速度 m/s
constexpr float IMPACT_V    = 0.45f;    // 撞击前进速度 (2026-08-13: 比WALK_V更猛, 朝球冲击)
constexpr float STEP_H      = 0.17f;    // 步高
constexpr float ENTER_DIST_M = 0.92f;   // 衔接: Stage1结束后 前进0.92m (2026-08-13)
constexpr float TURN_MAIN   = 90.0f;    // 转 90° (方向按动作)
constexpr float FWD_LONG_M  = 2.8f;     // 每轮前进主段 2.8m (2026-08-13: 2.9→2.8)
constexpr float FWD_SHORT_M = 0.75f;    // 回正后前进 0.75m (2026-08-13: 0.6→0.75)
constexpr float SCAN1_DEG   = +45.0f;   // 扫描位1: 左转 45°
constexpr float SCAN2_DEG   = -90.0f;   // 扫描位2: 右转 90°
constexpr float BACK_DEG    = +45.0f;   // 回正: 左转 45°
constexpr int   SCAN_WAIT_FRAMES = 200; // 每角度停 2 秒 (100Hz)
constexpr float SCAN_POKE_DIST   = 0.2f; // 撞击后退回 0.2m
constexpr float BALL_MAX_DIST    = 0.7f; // 橙色球距离 ≤0.7m 才算找到 (2026-08-12: 0.8→0.7)
constexpr float IMPACT_DIST      = 0.12f; // 撞击到位: 球距<0.12m (2026-08-13 距离闭环)
constexpr float IMPACT_MAX       = 0.5f;  // 撞击最大冲刺 0.5m (球到脚下/丢失保护)
constexpr int   SCAN_CONFIRM_FRAMES = 12; // 球连续确认12帧(~0.12s)才算数, 防误检 (2026-08-13)
constexpr float TURN_SPEED = 0.45f;    // 转向速度 (实测 yaw=0.6 实际≈2rad/s, 0.3太慢, 2026-08-12 定0.45≈1.5rad/s)
constexpr float TURN_DONE_ERR      = 0.04f;  // 转向完成误差 (2026-08-12: 0.05→0.04 + 停稳确认)
constexpr float TURN_SLOW_ERR      = 0.25f;  // 末期减速误差阈值, <0.25rad 半速 (2026-08-12 新增)
constexpr int   TURN_SETTLE_FRAMES = 25;     // 转到位停稳 0.25s (2026-08-12: 15→25 等abs_yaw稳定)
// ★ 转向精度补偿 (2026-08-13): 实测哪边偏小就填哪边, 不用再改角度主参数
constexpr float LEFT_TURN_COMP_DEG  = 0.0f;  // 左转固定补偿角(°), 实测左转偏小→填正(如+3)
constexpr float RIGHT_TURN_COMP_DEG = 0.0f;  // 右转固定补偿角(°), 实测右转偏小→填正
constexpr float TURN_FINE_ERR = 0.06f;  // 精调误差阈值, <0.06rad 用超慢速蠕行提高末端精度
constexpr float TURN_FINE_SPD = 0.12f;  // 精调速度 rad/s (超慢, 防过冲)

}  // namespace

void Stage2Real::init() {
    done_          = false;
    round_         = 0;
    start_yaw_     = sensor_.abs_yaw;
    turn_base_yaw_ = sensor_.abs_yaw;
    fwd_ref_yaw_   = sensor_.abs_yaw;
    turn_guard_    = 0;
    turn_settle_   = 0;
    wait_frames_   = 0;
    adjust_frames_ = 0;
    phase_         = Phase::FWD0;   // 开头衔接: 前进0.92m → 左转90° (2026-08-13)
    last_x_        = sensor_.odom_x;
    last_y_        = sensor_.odom_y;
    traveled_      = 0.0f;
    hold_x_        = sensor_.odom_x;
    hold_y_        = sensor_.odom_y;
    turn_start_x_  = sensor_.odom_x;
    turn_start_y_  = sensor_.odom_y;
    RCLCPP_INFO(rclcpp::get_logger("stage2_real"),
                "[Stage2Real] init: 衔接前进%.2fm+左转90° → 四轮S形(每轮前进%.1fm+扫描+回正+%.1fm)",
                ENTER_DIST_M, FWD_LONG_M, FWD_SHORT_M);
}

void Stage2Real::run() {
    if (done_) return;

    // ── 原地位置保持 (2026-08-13): 踏步会后退+odom漂移, 机器人系后退>2cm → 小前进顶回 ──
    //   (侧移不可靠, 只修前后分量; 每帧按 hold_x_/hold_y_ 基准计算)
    auto pos_hold = [&]() {
        float dx = sensor_.odom_x - hold_x_;
        float dy = sensor_.odom_y - hold_y_;
        float back = dx * std::cos(sensor_.abs_yaw) + dy * std::sin(sensor_.abs_yaw);
        if (back < -0.02f) {
            motion_.set_walk_velocity_step(0.08f, 0.0f, 0.0f, STEP_H);   // 后退>2cm → 顶回
        } else {
            motion_.set_walk_velocity_step(0.0f, 0.0f, 0.0f, STEP_H);
        }
    };

    // ── 转向: TURN0(衔接左转90°)/TURN1(轮首转)/TURN2(轮中转)/TURN3(回正) + SCAN1_TURN/SCAN2_TURN ──
    if (phase_ == Phase::TURN0 || phase_ == Phase::TURN1 || phase_ == Phase::TURN2 || phase_ == Phase::TURN3 ||
        phase_ == Phase::SCAN1_TURN || phase_ == Phase::SCAN2_TURN) {
        // 角度按动作 (2026-08-13 四轮S形):
        //   轮首转(轮2/3/4): 轮2右90° 轮3左90° 轮4右90° → 奇数轮右转
        //   轮中转(扫描前): 轮1右90° 轮2左90° 轮3右90° 轮4左90° → 偶数轮右转
        float deg;
        switch (phase_) {
            case Phase::TURN0: deg = TURN_MAIN; break;                              // 衔接左转90°
            case Phase::TURN1: deg = (round_ % 2 == 1) ? -TURN_MAIN : TURN_MAIN; break;
            case Phase::TURN2: deg = (round_ % 2 == 0) ? -TURN_MAIN : TURN_MAIN; break;
            case Phase::TURN3: deg = BACK_DEG; break;                            // 回正
            case Phase::SCAN1_TURN: deg = SCAN1_DEG; break;
            default:                deg = SCAN2_DEG; break;
        }
        // 左右转向固定补偿角 (2026-08-13): 实测偏小填正, 不用改主参数
        if (deg > 0.0f) deg += LEFT_TURN_COMP_DEG;
        else if (deg < 0.0f) deg -= RIGHT_TURN_COMP_DEG;
        float target_yaw = norm_yaw(turn_base_yaw_ + deg * M_PI / 180.0f);
        float yaw_err    = norm_yaw(target_yaw - sensor_.abs_yaw);

        // 转向开始时记录起点 (转向后顶回后退漂移, 2026-08-13)
        if (turn_guard_ == 0 && turn_settle_ == 0) {
            turn_start_x_ = sensor_.odom_x;
            turn_start_y_ = sensor_.odom_y;
        }

        // 转向完成 → 进入 ADJUST (朝转向起点顶回后退漂移), 修正完再切下一段
        auto finish_turn = [&]() {
            motion_.stop();
            switch (phase_) {
                case Phase::TURN0:      after_adjust_ = Phase::FWD1;       break;   // 衔接后直接前进2.8m(轮1无首转)
                case Phase::TURN1:      after_adjust_ = Phase::FWD1;       break;
                case Phase::TURN2:      after_adjust_ = Phase::SCAN1_TURN; break;
                case Phase::TURN3:      after_adjust_ = Phase::FWD3;       break;
                case Phase::SCAN1_TURN: after_adjust_ = Phase::SCAN1_WAIT; break;
                default:                after_adjust_ = Phase::SCAN2_WAIT; break;   // SCAN2_TURN
            }
            fwd_ref_yaw_ = target_yaw;   // 前进回正基准 (ADJUST 后使用)
            hold_x_ = turn_start_x_;     // 修正基准 = 转向起点
            hold_y_ = turn_start_y_;
            adjust_frames_ = 0;
            phase_ = Phase::ADJUST;
#ifdef DEBUG_STAGE
            float dx = sensor_.odom_x - hold_x_, dy = sensor_.odom_y - hold_y_;
            fprintf(stderr, "[S2Stage] 轮%d 转向%.0f°完成 err=%.2f 漂移=%.3fm, 修正中\n",
                    round_ + 1, deg, yaw_err, std::hypot(dx, dy));
            fflush(stderr);
#endif
        };

        // 停稳重校正 (2026-08-12): 转到位先停0.15s等abs_yaw稳定, 停稳后误差还大→低速补转
        if (turn_settle_ > 0) {
            pos_hold();   // 停稳期间也防后退 (2026-08-13)
            if (++turn_settle_ <= TURN_SETTLE_FRAMES) return;
            turn_settle_ = 0;
            if (std::abs(yaw_err) < TURN_DONE_ERR) { finish_turn(); return; }
            turn_guard_ = 0;                     // 还偏 → 低速补转
            return;
        }

        bool ok = (turn_guard_ > 20) && (std::abs(yaw_err) < TURN_DONE_ERR);  // 至少0.2s防跳变
        turn_guard_++;
        if (ok) {
            motion_.stop();
            turn_settle_ = 1;                    // 转到位 → 停稳确认
        } else {
            // 三级速度: 误差<0.06rad 超慢精调(0.12) → <0.25rad 半速 → 全速 (2026-08-13)
            float spd;
            if (std::abs(yaw_err) < TURN_FINE_ERR)      spd = TURN_FINE_SPD;
            else if (std::abs(yaw_err) < TURN_SLOW_ERR) spd = TURN_SPEED * 0.5f;
            else                                        spd = TURN_SPEED;
            motion_.set_walk_velocity_step(0.0f, 0.0f, yaw_err > 0 ? spd : -spd, STEP_H);
#ifdef DEBUG_STAGE
            if (turn_guard_ % 20 == 0) {
                fprintf(stderr, "[S2Stage] 转向中 目标%.0f° err=%.2f spd=%.2f\n", deg, yaw_err, spd);
                fflush(stderr);
            }
#endif
        }
        return;
    }

    // ── 转向后修正: 朝转向起点顶回后退漂移 (2026-08-13) ──
    //   原地转圈时踏步会产生后退, 转完先把后退量顶回来再走
    if (phase_ == Phase::ADJUST) {
        pos_hold();
        float dx = sensor_.odom_x - hold_x_;
        float dy = sensor_.odom_y - hold_y_;
        float back = dx * std::cos(sensor_.abs_yaw) + dy * std::sin(sensor_.abs_yaw);
        if (back > -0.02f || ++adjust_frames_ > 200) {   // 顶回到位 或 2s 超时保护
            phase_ = after_adjust_;
            switch (phase_) {
                case Phase::FWD1: case Phase::FWD3:
                    last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
                    traveled_ = 0.0f;
                    break;
                case Phase::SCAN1_WAIT: case Phase::SCAN2_WAIT:
                    wait_frames_ = 0;
                    hold_x_ = sensor_.odom_x; hold_y_ = sensor_.odom_y;   // 扫描原地保持基准
                    break;
                default:   // 下一个转向 (SCAN1_TURN)
                    turn_guard_    = 0;
                    turn_base_yaw_ = sensor_.abs_yaw;
                    break;
            }
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 修正完成 (漂移剩余=%.3fm)\n",
                    std::hypot(sensor_.odom_x - hold_x_, sensor_.odom_y - hold_y_));
            fflush(stderr);
#endif
        }
        return;
    }

    // ── 扫描停 2 秒找球 ──
    if (phase_ == Phase::SCAN1_WAIT || phase_ == Phase::SCAN2_WAIT) {
        pos_hold();   // 位置保持: 踏步会后退, 后退>2cm自动顶回 (2026-08-13)
        bool is_scan1 = (phase_ == Phase::SCAN1_WAIT);
        if (sensor_.ball_found && sensor_.ball_dist <= BALL_MAX_DIST) {
            // 连续确认12帧才触发撞击 (2026-08-13): 误检单帧闪动不算, 真球持续存在必然确认成功
            if (++ball_confirm_ >= SCAN_CONFIRM_FRAMES) {
                ball_confirm_ = 0;
                phase_ = is_scan1 ? Phase::SCAN1_ACT : Phase::SCAN2_ACT;
                last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
                traveled_ = 0.0f;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 扫描位%d 确认球(dist=%.2fm), 朝球冲击\n",
                        is_scan1 ? 1 : 2, sensor_.ball_dist);
                fflush(stderr);
#endif
            }
        } else {
            ball_confirm_ = 0;
            if (++wait_frames_ >= SCAN_WAIT_FRAMES) {   // 2秒无球 → 下一位
                if (is_scan1) {
                    phase_ = Phase::SCAN2_TURN;
                    turn_guard_    = 0;
                    turn_base_yaw_ = sensor_.abs_yaw;
                } else if (round_ < 3) {
                    phase_ = Phase::TURN3;                       // 回正左45°
                    turn_guard_    = 0;
                    turn_base_yaw_ = sensor_.abs_yaw;
                } else {
                    phase_ = Phase::DONE;                        // 轮4: 扫描完直接结束
                }
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 轮%d 扫描位%d 2秒无球, %s\n",
                        round_ + 1, is_scan1 ? 1 : 2,
                        is_scan1 ? "右转90°" : (round_ < 3 ? "回正左45°" : "结束"));
                fflush(stderr);
#endif
            }
        }
        return;
    }

    // ── 朝球冲击: 距离闭环, 球距<0.12m 到位; 球丢失保持方向; 0.5m保护 ──
    if (phase_ == Phase::SCAN1_ACT || phase_ == Phase::SCAN2_ACT) {
        bool is_scan1 = (phase_ == Phase::SCAN1_ACT);
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;
        traveled_ += moved;

        // 到位: 球距<0.12m → 退回; 保护: 冲了0.5m还没到(球被身体挡住/丢失) → 强停退回 (2026-08-13)
        bool reached = sensor_.ball_found && sensor_.ball_dist > 0.01f &&
                       sensor_.ball_dist < IMPACT_DIST;
        if (reached || traveled_ >= IMPACT_MAX) {
            motion_.stop();
            phase_ = is_scan1 ? Phase::SCAN1_BACK : Phase::SCAN2_BACK;
            last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
            traveled_ = 0.0f;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 撞击完成%s, 退回\n",
                    reached ? "(球距<0.12m)" : "(0.5m保护)");
            fflush(stderr);
#endif
        } else {
            // 朝球方向引导冲击: ball_x(-1左~1右), 球丢失时保持原方向继续
            float bx = sensor_.ball_found ? sensor_.ball_x : 0.0f;
            float ball_yaw = std::max(-0.25f, std::min(0.25f, -bx * 0.5f));
            motion_.set_walk_velocity_step(IMPACT_V, 0.0f, ball_yaw, STEP_H);
        }
        return;
    }

    // ── 退回 0.2m ──
    if (phase_ == Phase::SCAN1_BACK || phase_ == Phase::SCAN2_BACK) {
        bool is_scan1 = (phase_ == Phase::SCAN1_BACK);
        float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
        last_x_ = sensor_.odom_x; last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;
        traveled_ += moved;
        if (traveled_ >= SCAN_POKE_DIST) {
            motion_.stop();
            // 已撞击 → 跳过剩余扫描, 直接回正/结束 (2026-08-13 用户要求: 撞到就不扫了)
            if (round_ < 3) {
                phase_ = Phase::TURN3;                       // 回正
                turn_guard_    = 0;
                turn_base_yaw_ = sensor_.abs_yaw;
            } else {
                phase_ = Phase::DONE;                        // 轮4结束
            }
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 退回完成, 已撞击 → %s\n",
                    round_ < 3 ? "回正左45°(跳过剩余扫描)" : "结束");
            fflush(stderr);
#endif
        } else {
            motion_.set_walk_velocity_step(-WALK_V * 0.8f, 0.0f, 0.0f, STEP_H);  // 后退
        }
        return;
    }

    if (phase_ == Phase::DONE) { done_ = true; return; }

    // ── 前进 FWD0(衔接0.92m)/FWD1(2.8m)/FWD3(0.75m): 累计位移 + 跳变保护, 按 fwd_ref_yaw_ 回正 ──
    float dist  = (phase_ == Phase::FWD0) ? ENTER_DIST_M :
                  (phase_ == Phase::FWD1) ? FWD_LONG_M : FWD_SHORT_M;
    float moved = std::hypot(sensor_.odom_x - last_x_, sensor_.odom_y - last_y_);
    last_x_ = sensor_.odom_x;
    last_y_ = sensor_.odom_y;
    if (moved > 0.25f) moved = 0.0f;
    traveled_ += moved;

    if (traveled_ >= dist) {
        motion_.stop();
        if (phase_ == Phase::FWD0) {
            phase_ = Phase::TURN0;                        // 衔接前进完成 → 左转90°
            turn_guard_    = 0;
            turn_base_yaw_ = sensor_.abs_yaw;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 衔接前进%.2fm完成, 左转90°\n", ENTER_DIST_M);
            fflush(stderr);
#endif
        } else if (phase_ == Phase::FWD1) {
            phase_ = Phase::TURN2;                        // 每轮前进2.8m后 → 轮中转(扫描前)
            turn_guard_    = 0;
            turn_base_yaw_ = sensor_.abs_yaw;
#ifdef DEBUG_STAGE
            fprintf(stderr, "[S2Stage] 轮%d 走%.1fm完成, 轮中转\n", round_ + 1, FWD_LONG_M);
            fflush(stderr);
#endif
        } else {                             // FWD3 收尾 → 下一轮
            round_++;
            if (round_ < 4) {
                phase_ = Phase::TURN1;
                turn_guard_    = 0;
                turn_base_yaw_ = sensor_.abs_yaw;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 收尾%.1fm完成, 进入第%d轮\n", FWD_SHORT_M, round_ + 1);
                fflush(stderr);
#endif
            } else {
                phase_ = Phase::DONE;
#ifdef DEBUG_STAGE
                fprintf(stderr, "[S2Stage] 全部4轮完成, DONE\n");
                fflush(stderr);
#endif
            }
        }
        return;
    }

    // abs_yaw 定位噪声大(实测日志波动±0.1rad), 死区防抖 + 增益0.5 (2026-08-12)
    // ★ norm_yaw 必须: abs_yaw 跨过±π边界时(如左转93°后), 不norm会 yaw_err≈6rad → 转向饱和疯狂转 (2026-08-12 修)
    float yaw_err = norm_yaw(sensor_.abs_yaw - fwd_ref_yaw_);
    if (std::abs(yaw_err) < 0.03f) yaw_err = 0.0f;
    float yaw_cmd = std::max(-0.5f, std::min(0.5f, -yaw_err * 0.5f));
#ifdef DEBUG_MOTION
    static int dbg_m_ = 0;
    if (++dbg_m_ % 10 == 0) {
        fprintf(stderr, "[S2M] cmd v=(%.2f,0,%.2f) 步高%.2f\n", WALK_V, yaw_cmd, STEP_H);
        fflush(stderr);
    }
#endif
    motion_.set_walk_velocity_step(WALK_V, 0.0f, yaw_cmd, STEP_H);
}
