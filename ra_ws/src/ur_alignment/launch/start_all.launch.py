from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # 1. d435i camera node
        Node(
            package="uralignment_cpp",
            executable="d435i_camera",
            name="d435i_camera"
        ),

        # 2. apriltags node
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="uralignment_cpp",
                    executable="apriltags",
                    name="apriltags"
                )
            ]
        ),

        # 3. teleop_ur_key node
        TimerAction(
            period=6.0, 
            actions=[
                ExecuteProcess(
                    # open a new tab and run teleop; keep the tab open after exit:
                    cmd=['bash', '-lc',
                        'source /opt/ros/jazzy/setup.bash; '
                        'source ~/ros2_ws/install/setup.bash; '
                        'ros2 run uralignment_cpp teleop_ur_key; '
                        'exec bash'],
                    prefix='gnome-terminal --tab -- ',
                    output='screen'
                )
            ]
        ),

        # 4. ur_e_series node
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="uralignment_cpp",
                    executable="ur_e_series",
                    name="ur_e_series"
                )
            ]
        ),

        # 5. displays node
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package="uralignment_cpp",
                    executable="displays",
                    name="displays"
                )
            ]
        ),
    ])
