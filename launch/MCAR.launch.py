import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Spécifiez le chemin de votre fichier world personnalisé dans le répertoire 'worlds'
    world_file_path = "ROS2_Project/worlds/diff_drive.sdf"


    # Lancer Gazebo avec votre fichier world personnalisé
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {world_file_path}'  # Chargez le fichier SDF ici
        }.items(),
    )

    # Bridge pour lier les topics Gazebo à ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/vehicle_green/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/vehicle_green/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        parameters=[{
            'qos_overrides./model/vehicle_blue.subscriber.reliability': 'reliable',
            'qos_overrides./model/vehicle_green.subscriber.reliability': 'reliable'
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge
    ])
