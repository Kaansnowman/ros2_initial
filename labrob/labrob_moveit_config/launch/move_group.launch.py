import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config_dir = get_package_share_directory("labrob_moveit_config")
    
    # MoveItConfigsBuilder ile config klasöründeki parametreleri (OMPL, Kinematics vb.) topluyoruz
    moveit_configs = (
        MoveItConfigsBuilder("rascl", package_name="labrob_moveit_config")
        .robot_description(file_path="config/rascl/rascl.urdf")
        .robot_description_semantic(file_path="config/rascl/rascl.srdf")
        .trajectory_execution(file_path="config/rascl/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/rascl/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/rascl/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Ana MoveGroup düğümü (Arka plan planlama motoru)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_configs.to_dict()],
    )

    # Robot State Publisher (TF / eklem dönüşümlerini yayınlar)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_configs.robot_description],
    )

    # ros2_control_node (Donanım arayüzünü taşıyan ana sunucu nodu)
    ros2_controllers_path = os.path.join(
        moveit_config_dir, "config", "rascl", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    # İSTİSNA: Adamın hazırladığı load_ros2_controllers launch dosyasını buraya dahil ediyoruz
    load_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, "launch", "load_ros2_controllers.launch.py")
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            move_group_node,
            load_controllers_launch, # Sıralı yükleyici burada devreye giriyor
        ]
    )