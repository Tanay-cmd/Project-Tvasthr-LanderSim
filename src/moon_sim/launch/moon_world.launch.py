from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare('moon_sim'),
        'worlds',
        'moon_flat_world.sdf'  # or moon_flat_world.sdf
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', world_path],
            output='screen'
        ),
    
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/altimeter@ros_gz_interfaces/msg/Altimeter@gz.msgs.Altimeter'
            ],
            output='screen'
        ),

        
    # Node(
    #     package='moon_sim',
    #     executable='imu_reader.py',
    #     name='imu_reader',
    #     output='screen',
    #     ),
        
    # Node(
    #     package='moon_sim',
    #     executable='alti_reader.py',
    #     name='alti_reader',
    #     output='screen',
    # ),


    ])

    
