import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("bearmaxV2", package_name="v2_moveit_config2")
        .to_moveit_configs()
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("v2_moveit_config2"),
                'launch',
                'move_group.launch.py')
        )
    )

    use_rviz = LaunchConfiguration("use_rviz")

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("v2_moveit_config2"),
                "launch",
                "moveit_rviz.launch.py")
        ),
        condition=IfCondition(use_rviz)
    )

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("v2_moveit_config2"),
                'launch',
                'static_virtual_joint_tfs.launch.py')
        )
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("v2_moveit_config2"),
                'launch',
                'rsp.launch.py')
        )
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                moveit_config.package_path,
                "config",
                "ros2_controllers.yaml"
            )
        ]
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "all_group_controller",
        "joint_state_broadcaster"
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Hacky workaround to make arduino boot
    minicom = ExecuteProcess(
        cmd=["putty -load Ros2Test"],
        shell=True
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="False",
                description="Should Rviz be launched?"
            ),
            minicom,
            rviz_node,
            static_tf,
            rsp,
            move_group,
            ros2_control_node
        ]
        + load_controllers
    )
