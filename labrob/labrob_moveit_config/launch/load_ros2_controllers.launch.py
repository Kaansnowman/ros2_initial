#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the RASCL robot in a specific sequence.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # 1. Arm Controller'ı yükleme komutu
    start_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    # 2. Gripper Controller'ı yükleme komutu (Bizim yörünge tabanlı kontrolcümüz)
    start_gripper_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )

    # 3. Joint State Broadcaster'ı yükleme komutu
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Donanım arayüzünün (SDO/Simülasyon) tamamen hazır olması için 5 saniye bekleyip Broadcaster'ı başlatıyoruz
    delayed_start = TimerAction(
        period=5.0,
        actions=[start_joint_state_broadcaster_cmd]
    )

    # Sıralama: Joint State Broadcaster yüklenmesi tam bittiğinde Arm Controller'ı tetikle
    load_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_arm_controller_cmd]
        )
    )

    # Sıralama: Arm Controller yüklenmesi tam bittiğinde Gripper Controller'ı tetikle
    load_gripper_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_arm_controller_cmd,
            on_exit=[start_gripper_controller_cmd]
        )
    )

    ld = LaunchDescription()
    ld.add_action(delayed_start)
    ld.add_action(load_arm_controller_cmd)
    ld.add_action(load_gripper_controller_cmd)

    return ld