#!/usr/bin/env python3
"""
==============================================================================
BRINGUP LAUNCH FILE - NİHAİ SİSTEM BAŞLATICI
==============================================================================
Bu launch dosyası TÜM sistemi tek komutla başlatır:
  ✓ Gazebo simülasyonu (robot + dünya + kırmızı top)
  ✓ Robot State Publisher (URDF → TF ağacı)
  ✓ Joint State Publisher (tekerleklerin durumu)
  ✓ Static TF Publisher (world → odom)
  ✓ ROS-Gazebo köprüleri (kamera, cmd_vel, odom, tf, clock)
  ✓ RViz (görselleştirme + kamera görüntüsü)
  ✓ Ball Chaser node (top takip mantığı)
  ✓ TF Frames PDF çıktısı (otomatik oluşturulur ve açılır)

Kullanım:
    ros2 launch my_robot_bringup bringup.launch.py
==============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Tüm sistemi başlatan launch tanımı"""
    
    # ==================== PAKET YOLLARİ ====================
    my_robot_description_path = get_package_share_directory('my_robot_description')
    
    # ==================== URDF DOSYASI ====================
    urdf_file_path = os.path.join(my_robot_description_path, 'urdf', 'my_robot.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()
    
    # ==================== GAZEBO BAŞLATMA ====================
    # GPU hızlandırma ve render engine ayarlarıyla
    gazebo_launch = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', '-v', '4',
            os.path.join(my_robot_description_path, 'worlds', 'empty_world.sdf')
        ],
        additional_env={
            'GZ_SIM_RENDER_ENGINE': 'ogre',
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'
        },
        output='screen'
    )
    
    # ==================== STATIC TF PUBLISHER ====================
    # World → Odom frame dönüşümü (sabit)
    # RViz'de TF ağacının tam görünmesi için gerekli
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_broadcaster_world_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        parameters=[{'use_sim_time': True}]
    )
    
    # ==================== ROBOT STATE PUBLISHER ====================
    # URDF'den robot yapısını okur ve TF ağacını yayınlar
    # base_link → wheels, camera_link vb.
    # NOT: odom→base_link dönüşümünü Gazebo yayınlar, bu düğüm karışmaz
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True},
            {'publish_frequency': 50.0}  # Sadece URDF'deki jointleri yayınla
        ]
    )
    
    # ==================== ROBOT SPAWN ====================
    # Robotu Gazebo simülasyonuna ekler
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name', 'my_robot',
            '-allow_renaming', 'true',
            '-z', '0.08'  # Robotun başlangıç yüksekliği
        ]
    )
    
    # ==================== ROS-GAZEBO KÖPRÜLERİ ====================
    # Gazebo ↔ ROS2 iletişimini sağlar
    # - cmd_vel: ROS → Gazebo (robot kontrolü)
    # - odom: Gazebo → ROS (odometry verisi)
    # - tf: Gazebo → ROS (transform ağacı)
    # - camera: Gazebo → ROS (kamera görüntüsü)
    # - joint_states: Gazebo → ROS (tekerlek pozisyonları)
    # NOT: /clock köprüsü YOK - Gazebo'nun RosClockPublisher plugin'i kullanılıyor
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
            # NOT: /clock yok! Gazebo'nun kendi RosClockPublisher plugin'i kullanılıyor
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # ==================== GECİKMELİ BAŞLATMA ====================
    # Gazebo'nun tamamen hazır olması için 5 saniye bekle
    # Sonra tüm bağımlı node'ları başlat
    start_all_after_gazebo = TimerAction(
        period=5.0,
        actions=[
            # Robot tanımı ve spawn (5s)
            robot_state_publisher_node,
            spawn_entity_node,
            static_tf_world_odom,
            bridge_node,
            # NOT: joint_state_publisher_gui KALDIRILDI!
            # Gazebo'nun DiffDrive plugin'i zaten /joint_states yayınlıyor
            # İki publisher çakışıyor ve robot_state_publisher kafası karışıyordu
        ]
    )
    
    # RViz (8s)
    start_rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(my_robot_description_path, 'rviz', 'display_robot.rviz')],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Ball Chaser (10s) - KIRMIZI TOPU TAKİP EDER
    start_ball_chaser = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='my_robot_chaser',
                executable='ball_chaser',
                name='ball_chaser',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # TF Frames PDF (15s)
    start_tf_frames = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'cd ~ && ros2 run tf2_tools view_frames && sleep 2 && evince $(ls -t ~/frames_*.pdf | head -1) &'
                ],
                output='screen'
            )
        ]
    )
    
    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        # 0s: SADECE Gazebo başlar
        gazebo_launch,
        
        # 5s: Robot spawn + köprüler
        start_all_after_gazebo,
        
        # 8s: RViz
        start_rviz,
        
        # 10s: Ball Chaser - KIRMIZI TOPU TAKİP EDER! 🔴
        start_ball_chaser,
        
        # 15s: TF Frames PDF
        start_tf_frames,
    ])
