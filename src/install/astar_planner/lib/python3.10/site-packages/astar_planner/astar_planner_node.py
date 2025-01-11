import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)  # Publicar cada 0.5 segundos
        self.get_logger().info('Nodo A* Planner iniciado.')
        self.goal_position = Point()
        self.goal_position.x = 5.0
        self.goal_position.y = 0.0
        self.goal_position.z = 0.0

    def publish_marker(self):
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
