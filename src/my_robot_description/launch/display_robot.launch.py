import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paketin paylaşım dizinini al
    my_robot_description_path = get_package_share_directory('my_robot_description')
    urdf_file_path = os.path.join(my_robot_description_path, 'urdf', 'my_robot.urdf')

    # URDF dosyasının içeriğini oku
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher düğümü
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # Joint State Publisher düğümü
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Static TF Publisher: Sadece world -> odom için
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_broadcaster_world_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )


    # RViz düğümü
    rviz_config_dir = os.path.join(my_robot_description_path, 'rviz')
    default_rviz_config_path = os.path.join(rviz_config_dir, 'display_robot.rviz')

    # RViz yapılandırma dosyasını belirtmek için launch argümanı
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )
    
    # RViz klasörünü de oluşturmamız gerekecek
    # mkdir -p ~/myrobb_ws/src/my_robot_description/rviz
    # touch ~/myrobb_ws/src/my_robot_description/rviz/display_robot.rviz
    # İlk çalıştırmada RViz ayarlarını kaydedip bu dosyayı güncelleyeceğiz.

    return LaunchDescription([
        declare_rviz_config_file_cmd,
        static_tf_world_odom,     
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
