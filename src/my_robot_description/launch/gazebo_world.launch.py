import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Paketlerin paylaşım dizinlerini al
    my_robot_description_path = get_package_share_directory('my_robot_description')

    # URDF dosyasının yolunu al (my_robot.urdf kullanılıyor)
    urdf_file_path = os.path.join(my_robot_description_path, 'urdf', 'my_robot.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo'yu başlatma
    gazebo_launch = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r',
            '--render-engine', 'ogre',
            os.path.join(my_robot_description_path, 'worlds', 'empty_world.sdf')
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # Robotu Gazebo'da Spawn Etme
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'my_robot',
                   '-allow_renaming', 'true',
                   '-z','0.1']
    )
    
    # ROS-Gazebo Köprüsü (Kamera için)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ]
    )  

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        bridge_node
    ])
