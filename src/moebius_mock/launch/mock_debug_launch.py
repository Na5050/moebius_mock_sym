import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition # Импортируем оба движка условий
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
    # *. Объявляем ОДИН аргумент переключения
    # По умолчанию он 'false', то есть активирует дефолтную ноду
    declare_switch_arg = DeclareLaunchArgument(
        'use_lite',
        default_value='false',
        choices=['true', 'false'],
        description='Переключить запуск на облегченный симулятор (true/false)'
    )

    # ВВОДИМ НОВЫЙ ФЛАГ: По умолчанию 'false' (rviz выключен)
    declare_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        choices=['true', 'false'],  # Разрешаем вводить только true или false
        description='Запустить ноду rviz вместе с симуляцией (true/false)'
    )

    # ВВОДИМ НОВЫЙ ФЛАГ: По умолчанию 'false' (автопилот выключен)
    declare_autopilot_arg = DeclareLaunchArgument(
        'start_autopilot',
        default_value='false',
        choices=['true', 'false'],  # Разрешаем вводить только true или false
        description='Запустить ноду автопилота вместе с симуляцией (true/false)'
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
        parameters=[config_file_path],
        condition=UnlessCondition(LaunchConfiguration('use_lite')) # Запуск, если НЕ true
    )

    # Нода симулятора-заглушки (ПРИНИМАЕТ ДИНАМИЧЕСКИЙ ПУТЬ)
    mock_robot_lite_node = Node(
        package=mock_package,
        executable='mock_moebius_maze_opencv.py',
        name='mock_moebius_maze',
        output='screen',
        parameters=[config_file_path],
        condition=IfCondition(LaunchConfiguration('use_lite')) # Запуск, если true
    )

    # Публикатор TF трансформаций
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            # Частота публикации TF в Гц (даже статических)
            'publish_frequency': 30.0 
        }]
    )

    # Визуализатор RViz2
    cam_vw_node = Node(
        package=mock_package,
        executable='camera_viewer.py',
        name='camera_viewer',
        output='screen',
        # Используем ParameterValue для принудительной типизации bool
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(LaunchConfiguration('start_rviz')),
        respawn=True,       # <--- ОБЯЗАТЕЛЬНО ДОБАВИТЬ: автоматически перезапустит ноду при вылете
        respawn_delay=2.0 
    )

    # Визуализатор RViz2
    map_vw_node = Node(
        package=mock_package,
        executable='map_odom_viewer.py',
        name='map_odom_viewer',
        output='screen',
        # Используем ParameterValue для принудительной типизации bool
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(LaunchConfiguration('start_rviz')),
        respawn=True,       # <--- ОБЯЗАТЕЛЬНО ДОБАВИТЬ: автоматически перезапустит ноду при вылете
        respawn_delay=2.0
    )

    # Визуализатор RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # Используем ParameterValue для принудительной типизации bool
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        respawn=True,       # <--- ОБЯЗАТЕЛЬНО ДОБАВИТЬ: автоматически перезапустит ноду при вылете
        respawn_delay=2.0
    )

    # Ваша нода автопилота
    autopilot_node = Node(
        package=autopilot_package,
        executable='autopilot_node.py',        
        name='robot_autopilot',                
        output='screen',
        parameters=[{'use_sim_time': True}],
        # МАНЕВР: Нода запустится, только если LaunchConfiguration равен 'true'
        condition=IfCondition(LaunchConfiguration('start_autopilot'))
    )

    return LaunchDescription([
        declare_switch_arg,
        declare_scene_arg, # Обязательно регистрируем аргумент в лаунчере
        declare_autopilot_arg,  # Регистрируем аргумент автопилота
        declare_rviz_arg,       # Регистрируем аргумент RViz2
        mock_robot_node,
        mock_robot_lite_node,
        robot_state_publisher_node,
        cam_vw_node,
        map_vw_node,
        rviz_node,
        autopilot_node
    ])
