#include "cyberdog_race/race_controller.hpp"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <execinfo.h>

// ── 崩溃信号处理：打印调用栈后退出 ──
static void crash_handler(int sig) {
    fprintf(stderr, "\n=== CRASH signal %d ===\n", sig);
    void* frames[32];
    int n = backtrace(frames, 32);
    backtrace_symbols_fd(frames, n, STDERR_FILENO);
    _exit(sig);
}

int main(int argc, char** argv) {
    signal(SIGSEGV, crash_handler);
    signal(SIGABRT, crash_handler);
    signal(SIGBUS, crash_handler);
    signal(SIGFPE, crash_handler);
    // ★ SIGPIPE 必须忽略：浏览器断开连接后，旧推流线程 write() 会触发 SIGPIPE，
    //   默认动作是终止整个进程（不是 SIGSEGV，crash_handler 捕不到）→ "切换画面就崩溃"
    signal(SIGPIPE, SIG_IGN);

    rclcpp::init(argc, argv);
    try {
    auto node = std::make_shared<RaceController>();

    fprintf(stderr, "[MAIN] entering spin...\n");
    rclcpp::spin(node);
    fprintf(stderr, "[MAIN] spin RETURNED!\n");

    // Ctrl+C后发停止指令再坐下
    auto& motion = node->get_motion();
    for (int i = 0; i < 20; i++) {
        motion.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stand();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    motion.lie_down();
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "[FATAL] unknown exception" << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
