from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo del planificador A*
        Node(
            package='astar_planner',  # Nombre del paquete
            executable='astar_planner_node',  # Nombre del archivo ejecutable
            name='astar_planner',
            output='screen'
        ),
        # Nodo del controlador
        Node(
            package='astar_planner',  # Nombre del paquete
            executable='controller',  # Nombre del archivo ejecutable
            name='controller',
            output='screen'
        ),
        # Nodo del planificador general
        Node(
            package='astar_planner',  # Nombre del paquete
            executable='planner',  # Nombre del archivo ejecutable
            name='planner',
            output='screen'
        ),
    ])
