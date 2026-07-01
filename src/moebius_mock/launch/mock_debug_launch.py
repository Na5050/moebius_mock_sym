import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    mock_package = 'moebius_mock'
    autopilot_package = 'robot_autopilot' 
    
    # 1. Объявляем аргумент терминала 'scene'
    # По умолчанию, если ничего не ввести, загрузится 'maze_scene.yaml'
    declare_scene_arg = DeclareLaunchArgument(
        'scene',
        default_value='maze_scene.yaml',
        description='Имя YAML-файла сценария лабиринта из папки config'
    )

    # 2. Динамически собираем путь к файлу конфигурации на основе введенного имени
    config_file_path = PathJoinSubstitution([
        get_package_share_directory(mock_package),
        'config',
        LaunchConfiguration('scene')
    ])

    # Путь к URDF-файлу описания робота
    urdf_file = os.path.join(get_package_share_directory(mock_package), 'urdf', 'moebius_mock.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Нода симулятора-заглушки (ПРИНИМАЕТ ДИНАМИЧЕСКИЙ ПУТЬ)
    mock_robot_node = Node(
        package=mock_package,
        executable='mock_moebius_maze.py',
        name='mock_moebius_maze',
        output='screen',
        parameters=[config_file_path] # Путь подставится автоматически во время старта
    )

    # Публикатор TF трансформаций
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Визуализатор RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Ваша нода автопилота
    autopilot_node = Node(
        package=autopilot_package,
        executable='autopilot_node.py',        
        name='robot_autopilot',                
        output='screen',
        parameters=[{'use_sim_time': True}]   
    )

    return LaunchDescription([
        declare_scene_arg, # Обязательно регистрируем аргумент в лаунчере
        mock_robot_node,
        robot_state_publisher_node,
        #rviz_node,
        autopilot_node
    ])
