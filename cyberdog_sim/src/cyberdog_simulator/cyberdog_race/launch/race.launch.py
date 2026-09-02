#!/usr/bin/env python3
# ============================================================
# race.launch.py — cyberdog_race 运行期版本选择入口
#
# ★ 2026-09-02 重构：替代 build_bins.sh 的 sed 多二进制机制
#   · 一个二进制（全编）→ launch 传参选版本，无需重编/换二进制
#   · Stage4 路线实现选择：stage4_impl := formal | test | test2
#
# 用法（NX 上）：
#   ros2 launch cyberdog_race race.launch.py stage4_impl:=test
#   ros2 launch cyberdog_race race.launch.py stage4_impl:=formal \
#       namespace:=/mi_desktop_48_b0_2d_7b_02_c7
#
# 等价命令行（不 launch 时）：
#   ros2 run cyberdog_race race_controller --ros-args \
#       -r __ns:=/mi_desktop_48_b0_2d_7b_02_c7 -p stage4_impl:=test
# ============================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'stage4_impl',
            default_value='formal',
            description='Stage4 真机路线实现: formal(正式) | test(test路线) | test2(test2路线)',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS2 命名空间 (真机如 /mi_desktop_48_b0_2d_7b_02_c7, 留空=全局)',
        ),
        Node(
            package='cyberdog_race',
            executable='race_controller',
            name='race_controller',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'stage4_impl': LaunchConfiguration('stage4_impl'),
            }],
        ),
    ])
