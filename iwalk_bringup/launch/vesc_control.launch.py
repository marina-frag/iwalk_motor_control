from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _as_bool(value, name):
    lowered = value.lower()
    if lowered in ('true', '1'):
        return True
    if lowered in ('false', '0'):
        return False
    raise ValueError(f'{name} must be true or false, got {value!r}')


def _setup(context):
    bringup_share = Path(get_package_share_directory('iwalk_bringup'))
    description_share = Path(
        get_package_share_directory('iwalk_description'))
    single_motor_text = LaunchConfiguration('single_motor').perform(context)
    single_motor = _as_bool(single_motor_text, 'single_motor')
    mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            'can_interface',
            'single_motor',
            'monitor_only',
            'feedback_timeout',
            'left_vesc_id',
            'right_vesc_id',
            'left_direction',
            'right_direction',
        )
    }
    robot_xml = xacro.process_file(
        str(description_share / 'urdf' / 'iwalk.urdf.xacro'),
        mappings=mappings,
    ).toxml()
    config_name = (
        'controllers_single.yaml' if single_motor
        else 'controllers_diff.yaml'
    )
    config = str(bringup_share / 'config' / config_name)
    controller = (
        'test_velocity_controller' if single_motor
        else 'diff_drive_controller'
    )
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_xml}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config, {'robot_description': robot_xml}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '-c', '/controller_manager',
                '-p', config,
                '--controller-manager-timeout', '60',
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller,
                '-c', '/controller_manager',
                '-p', config,
                '--controller-manager-timeout', '60',
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='vcan0'),
        DeclareLaunchArgument('single_motor', default_value='true'),
        DeclareLaunchArgument('monitor_only', default_value='false'),
        DeclareLaunchArgument('feedback_timeout', default_value='0.2'),
        DeclareLaunchArgument('left_vesc_id', default_value='1'),
        DeclareLaunchArgument('right_vesc_id', default_value='2'),
        DeclareLaunchArgument('left_direction', default_value='1'),
        DeclareLaunchArgument('right_direction', default_value='1'),
        OpaqueFunction(function=_setup),
    ])
