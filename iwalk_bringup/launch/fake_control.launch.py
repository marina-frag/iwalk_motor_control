from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('iwalk_bringup'))
    config = str(share / 'config' / 'controllers.yaml')
    robot_xml = xacro.process_file(
        str(share / 'urdf' / 'iwalk_control.urdf.xacro')
    ).toxml()

    actions = []

    for motor_id in (1, 2):
        namespace = f'motor{motor_id}'

        actions.extend([
            Node(
                package='iwalk_sim_motor',
                executable='motor_node',
                namespace=namespace,
                output='screen',
            ),
            Node(
                package='iwalk_fake_encoder',
                executable='encoder_node',
                namespace=namespace,
                parameters=[{'counts_per_revolution': 4096}],
                output='screen',
            ),
            Node(
                package='iwalk_fake_vesc',
                executable='vesc_node',
                namespace=namespace,
                parameters=[{
                    'can_interface': 'vcan0',
                    'vesc_id': motor_id,
                    'pole_pairs': 7,
                    'counts_per_revolution': 4096,
                }],
                output='screen',
            ),
        ])

    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_xml}],
        output='screen',
    ))

    # Give the fake devices time to begin publishing feedback.
    # The hardware plugin additionally checks freshness during activation.
    actions.append(TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[config],
                remappings=[('robot_description', '/robot_description')],
                output='screen',
            ),
        ],
    ))

    actions.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/controller_manager',
            '-p', config,
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    ))

    actions.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '-c', '/controller_manager',
            '-p', config,
            '--controller-manager-timeout', '60',
            '--controller-ros-args',
            '--ros-args -r ~/cmd_vel:=/cmd_vel -r ~/odom:=/odom',
        ],
        output='screen',
    ))

    return LaunchDescription(actions)
