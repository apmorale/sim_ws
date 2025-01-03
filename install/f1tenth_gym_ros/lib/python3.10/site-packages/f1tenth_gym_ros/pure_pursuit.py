import rclpy
import math
import csv
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # Crear suscripción al tópico de Odometry
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Leer waypoints desde el archivo CSV
        self.waypoints = self.read_waypoints_from_csv('/home/ana/sim_ws/waypoints.csv')
        self.current_pose = None
        self.lookahead_distance = 0.4  # Distancia de mirada inicial (puede ajustarse dinámicamente)

        self.get_logger().info("Pure Pursuit Controller inicializado")

    def read_waypoints_from_csv(self, file_path):
        waypoints = []
        with open(file_path, 'r') as file:
            reader = csv.reader(file, delimiter=',')  # Cambiar delimitador a coma
            next(reader)
            for row in reader:
                waypoints.append((float(row[0]), float(row[1])))
        return waypoints

    def odom_callback(self, msg):
        # Obtener la posición y orientación actuales del vehículo
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Convertir orientación quaternion a ángulo en radianes
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_pose = (current_x, current_y, current_yaw)
        self.pure_pursuit_control()

    def pure_pursuit_control(self):
        if not self.current_pose or not self.waypoints:
            return

        current_x, current_y, current_yaw = self.current_pose

        # Buscar el punto de mirada hacia adelante
        target_x, target_y = self.find_lookahead_point(current_x, current_y, current_yaw)

        # Calcular ángulo y velocidad angular
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        heading_error = angle_to_target - current_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # Normalizar

        # Calcular velocidad angular y lineal
        linear_velocity = 0.85  # Ajustable
        angular_velocity = 0.5 * heading_error  # Ganancia proporcional

        # Imprimir valores clave para diagnóstico
        self.get_logger().info(f"Current Position: x={current_x:.3f}, y={current_y:.3f}, yaw={current_yaw:.3f}")
        self.get_logger().info(f"Target Point: x={target_x:.3f}, y={target_y:.3f}")
        self.get_logger().info(f"Heading Error: {heading_error:.3f} radians")
        self.get_logger().info(f"Linear Velocity: {linear_velocity:.3f} m/s")
        self.get_logger().info(f"Angular Velocity: {angular_velocity:.3f} rad/s")

        # Publicar los comandos de velocidad
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)

    def find_lookahead_point(self, current_x, current_y, current_yaw):
        """
        Encuentra el punto más cercano que cumple la condición de distancia de mirada hacia adelante.
        """
        for i, waypoint in enumerate(self.waypoints):
            distance = math.sqrt((waypoint[0] - current_x) ** 2 + (waypoint[1] - current_y) ** 2)
            self.get_logger().info(f"Distance: {distance:.3f}")
            
            # Verifica si el waypoint está suficientemente lejos y en la dirección correcta
            if distance >= self.lookahead_distance:
                # Además, podemos comparar la dirección hacia el waypoint con la orientación del robot (yaw)
                dx = waypoint[0] - current_x
                dy = waypoint[1] - current_y
                angle_to_waypoint = math.atan2(dy, dx)
                angle_diff = abs(current_yaw - angle_to_waypoint)
                
                # Si el ángulo es suficientemente pequeño, significa que el waypoint está adelante
                if angle_diff < math.pi / 2:  # Limitar a 90 grados
                    return waypoint
        
        # Si no se encuentra un punto suficientemente lejos, regresar el último
        return self.waypoints[-1]


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()