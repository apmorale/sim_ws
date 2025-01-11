import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
from astar_planner.astar import AStar  # Importa tu clase AStar existente

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
        # Suscripciones
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Publicaciones
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Variables
        self.grid = None  # Inicializa la variable
        self.astar = None  # Inicializa AStar como None hasta que se asigne un grid
        self.map_data = None
        self.goal_pose = None
        self.start_pose = None  # Asumimos que el robot envía su posición actual

    def map_callback(self, msg):
        self.map_data = msg
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.get_logger().info("Mapa recibido.")

        # Inicializar A* solo cuando el mapa está disponible
        if self.grid is not None:
            self.astar = AStar(self.grid)
            self.get_logger().info("AStar inicializado.")
        else:
            self.get_logger().warn("Grid no recibido correctamente.")
        
    def goal_callback(self, msg):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Objetivo recibido: {self.goal_pose}")
        self.plan_path()

    def plan_path(self):
        if self.map_data is None or self.goal_pose is None or self.astar is None:
            self.get_logger().warning("No se puede planificar: falta mapa, objetivo o AStar.")
            return
        
        # Convertir el mapa a numpy
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position
        
        map_array = np.array(self.map_data.data, dtype=np.int8).reshape((height, width))
        
        # Convertir posiciones reales a índices del mapa
        start_idx = (
            int((self.start_pose[1] - origin.y) / resolution),
            int((self.start_pose[0] - origin.x) / resolution)
        )
        goal_idx = (
            int((self.goal_pose[1] - origin.y) / resolution),
            int((self.goal_pose[0] - origin.x) / resolution)
        )
        
        # Ejecutar A*
        path = self.astar.astar(start_idx, goal_idx, treshold=50)  # Ajusta el umbral según tu mapa
        
        if path is None:
            self.get_logger().error("No se encontró un camino.")
            return
        
        # Publicar el camino
        planned_path = Path()
        planned_path.header = Header()
        planned_path.header.frame_id = self.map_data.header.frame_id
        
        for idx in path:
            pose = PoseStamped()
            pose.header = planned_path.header
            pose.pose.position.x = idx[1] * resolution + origin.x
            pose.pose.position.y = idx[0] * resolution + origin.y
            pose.pose.position.z = 0.0
            planned_path.poses.append(pose)
        
        self.path_pub.publish(planned_path)
        self.get_logger().info("Camino planeado publicado.")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
