import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Paketlerin paylaşım dizinlerini al
    my_robot_description_path = get_package_share_directory('my_robot_description')
    gz_sim_ros_path = get_package_share_directory('ros_gz_sim')

    # URDF dosyasının yolunu al
    urdf_file_path = os.path.join(my_robot_description_path, 'urdf', 'my_robot.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo'yu başlatma
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_ros_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(my_robot_description_path, 'worlds', 'empty_world.sdf'),
            'gz_sim_system_plugin_path': '/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_broadcaster_world_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    # Robotu Gazebo'da Spawn Etme
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'my_robot',
                   '-allow_renaming', 'true',
                   '-z','0.08'] #yerin üzerinde başlaması için
    )
    
    # ROS-Gazebo Köprüsü (Bridge)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Kamera Görüntüsü (Gazebo -> ROS)
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            # Odometri (Gazebo -> ROS)
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # TF (Gazebo -> ROS)
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            # /cmd_vel (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        static_tf_world_odom,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
        bridge_node
    ])
