#include "cyberdog_race/race_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RaceController>();

    rclcpp::spin(node);

    // Ctrl+C后发停止指令再坐下
    auto& motion = node->get_motion();
    for (int i = 0; i < 20; i++) {
        motion.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    motion.stand();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    motion.lie_down();

    rclcpp::shutdown();
    return 0;
}
