import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # --- CONFIGURACIÓN DE RUTAS ---
    package_name = 'camera_subscriber'
    
    # Ruta del mapa
    map_file_path = '/home/administrador/aruco_ws/src/map2gazebo/mapa_editado.yaml'
    
    # Ruta del script de YOLO
    yolo_script_path = '/home/administrador/aruco_ws/src/silla_detector/silla_detector/detector_node.py'
    
    # Ruta del entorno virtual de YOLO
    venv_activate_path = '/home/administrador/ros_yolo_env/bin/activate'

    map_yaml_file = LaunchConfiguration('map')

    return LaunchDescription([
        
        # Argumento para poder cambiar el mapa desde terminal si quisieras
        DeclareLaunchArgument(
            'map',
            default_value=map_file_path,
            description='Ruta completa al archivo yaml del mapa'),

        # ---------------------------------------------------------
        # BLOQUE 1: NAVEGACIÓN Y MAPA (Comandos 1, 2, 3)
        # ---------------------------------------------------------
        
        # 1. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]),

        # 2. Lifecycle Manager (para activar el mapa automáticamente)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),

        # 3. Static Transform (Map -> Odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'),

        # ---------------------------------------------------------
        # BLOQUE 2: VISIÓN (Comandos 4, 5)
        # ---------------------------------------------------------
        
        # 4. Driver IP (o Video)
        Node(
            package=package_name,
            executable='controlador_video',
            name='controlador_video',
            output='screen'),

        # 5. Detector de Marcadores ArUco
        Node(
            package=package_name,
            executable='detectar_markers',
            name='detectar_markers',
            output='screen'),

        # ---------------------------------------------------------
        # BLOQUE 3: YOLO CON ENTORNO VIRTUAL (Comando 6)
        # ---------------------------------------------------------
        # Como requiere activar un venv, usamos ExecuteProcess para correr un comando bash complejo.
        # "source activate && python script"
        ExecuteProcess(
            cmd=[[
                f'source {venv_activate_path} && ',
                'export PYTHONUNBUFFERED=1 && ',  # Para ver los prints en tiempo real
                f'python3 {yolo_script_path}'
            ]],
            shell=True,
            output='screen'
        ),

        # ---------------------------------------------------------
        # BLOQUE 4: VISUALIZACIÓN Y LÓGICA (Comandos 7, 8, 9)
        # ---------------------------------------------------------

        # 7. Fusión de Topics (ArUco + Estado)
        Node(
            package=package_name,
            executable='fusion_topics',
            name='fusion_topics',
            output='screen'),

        # 8. Visualizador de Mesas (Marcadores en RViz)
        Node(
            package=package_name,
            executable='visualizador_mesas',
            name='visualizador_mesas',
            output='screen'),

        # 9. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # Si tienes una config guardada, descomenta la siguiente línea y pon la ruta:
            # arguments=['-d', '/home/administrador/mi_config.rviz'],
            output='screen'
        )
    ])
