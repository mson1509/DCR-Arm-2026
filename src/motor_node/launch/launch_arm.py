from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Joystick node
    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        #parameters=[{'autorepeat_rate': 0.0}],
        output='screen'
    ))

    # SocketCAN bridge
    ld.add_action(Node(
        package='nobleo_socketcan_bridge',
        executable='socketcan_bridge',
        name='socketcan_bridge',
        output='screen'
    ))

    # ld.add_action(Node(
    #         package='motor_node',
    #         executable='start_motor',
    #         name='motor',
    #         output='screen'
    # ))

    # # Motor nodes (IDs 1 to 6)
    # for i in range(1, 7):
    #     ld.add_action(Node(
    #         package='motor_node',
    #         executable='start_motor',
    #         name=f'motor_{i}',
    #         parameters=[{'id': i}],
    #         output='screen'
    #     ))

    # Motor controller
    ld.add_action(Node(
        package='motor_node',
        executable='start_controller',
        name='motor_controller',
        output='screen'
    ))

    return ld
