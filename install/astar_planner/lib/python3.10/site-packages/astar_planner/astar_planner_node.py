import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from astar_planner.astar import AStar  # Importar la clase AStar desde el archivo astar.py

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner_node')

        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)  # Publicar cada 0.5 segundos
        self.get_logger().info('Nodo A* Planner iniciado.')

        # Definir el grid de ejemplo (esto se puede configurar desde otro lugar o cargar desde parámetros)
        self.grid = np.zeros((10, 10))  # 0 para espacios libres, otros valores para obstáculos
        self.astar = None  # AStar será inicializado en el callback de los datos del mapa

        # Definir puntos de inicio y fin para A* (puedes configurarlos como parámetros o valores fijos)
        self.start = (0, 0)
        self.goal = (9, 9)

        # Inicialización de la A* y planificación del camino
        self.path = None
        self.goal_position = None
        self.compute_path()

    def compute_path(self):
        # Inicializar AStar y calcular el camino
        self.astar = AStar(self.grid)  # Crear una nueva instancia de AStar con el grid
        self.path = self.astar.astar(self.start, self.goal)
        
        # Verificar si se encontró un camino
        if self.path:
            self.get_logger().info(f'Path found: {self.path}')
            # Utilizar la última posición del camino para el marcador
            self.goal_position = Point()
            self.goal_position.x = float(self.path[-1][0])  # Última posición del camino
            self.goal_position.y = float(self.path[-1][1])  # Última posición del camino
            self.goal_position.z = 0.0
        else:
            self.get_logger().info('No path found')
            self.goal_position = Point()
            self.goal_position.x = 5.0  # Valor por defecto si no se encuentra el camino
            self.goal_position.y = 0.0
            self.goal_position.z = 0.0

    def publish_marker(self):
        # Crear el marcador
        marker = Marker()
        marker.header.frame_id = "map"  # Cambiar si usas otro marco de referencia
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "astar_goal"
        marker.id = 0
        marker.type = Marker.SPHERE  # Tipo de marcador (puedes cambiar a CUBE, ARROW, etc.)
        marker.action = Marker.ADD

        # Posición del marcador
        marker.pose.position.x = self.goal_position.x
        marker.pose.position.y = self.goal_position.y
        marker.pose.position.z = self.goal_position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Escala del marcador
        marker.scale.x = 0.2  # Asegúrate de que sea visible
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Color del marcador
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Transparencia (1.0 = completamente visible)

        # Tiempo de vida del marcador
        marker.lifetime.sec = 0  # 0 = infinito

        # Publicar el marcador
        self.publisher_.publish(marker)
        self.get_logger().info(f'Marcador publicado en posición: {self.goal_position}')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
