import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import math

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        # Publicador para enviar comandos de velocidad
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Suscripción al topic /odom para obtener la posición del robot
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Pose inicial del robot (x, y, yaw)
        self.robot_pose = None  # Se actualizará en el callback de odometría

        # Parámetros de control
        self.linear_speed = 0.5  # Velocidad lineal fija
        self.angular_speed = 1.0  # Velocidad angular fija

        # Tolerancias para considerar que el objetivo se ha alcanzado
        self.goal_tolerance = 0.1  # Distancia mínima al objetivo
        self.yaw_tolerance = 0.05  # Tolerancia angular en radianes

        self.get_logger().info("Controller node initialized.")

    def odom_callback(self, msg):
        """
        Callback para el topic /odom. Actualiza la pose actual del robot.
        """
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convertir quaternion a ángulos de Euler para obtener yaw
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = R.from_quat(quaternion).as_euler('xyz')
        yaw = euler[2]  # Obtenemos el ángulo yaw

        # Guardar la pose como (x, y, yaw)
        self.robot_pose = (position.x, position.y, yaw)
        self.get_logger().debug(f"Updated robot pose: x={position.x}, y={position.y}, yaw={yaw:.2f} rad")

    def get_robot_pose(self):
        """
        Devuelve la pose actual del robot (x, y, yaw). Si no está disponible, muestra un warning.
        """
        if self.robot_pose is not None:
            return self.robot_pose
        else:
            self.get_logger().warning('Robot pose not available yet!')
            return None

    def move_to_goal(self, goal_x, goal_y):
        """
        Controlador para mover el robot a un objetivo (goal_x, goal_y).
        """
        if self.robot_pose is None:
            self.get_logger().error('Robot pose not available. Cannot move to goal.')
            return
        
        current_x, current_y, current_yaw = self.robot_pose

        # Calcular la distancia y el ángulo hacia el objetivo
        error_x = goal_x - current_x
        error_y = goal_y - current_y

        distance = math.sqrt(error_x**2 + error_y**2)  # Distancia al objetivo
        goal_angle = math.atan2(error_y, error_x)  # Ángulo hacia el objetivo
        angle_error = goal_angle - current_yaw  # Error angular

        # Normalizar el error angular entre -pi y pi
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Crear mensaje Twist para el control
        twist = Twist()

        if distance > self.goal_tolerance:
            # Control lineal y angular
            twist.linear.x = self.linear_speed * (1 if abs(angle_error) < math.pi / 4 else 0)
            twist.angular.z = self.angular_speed * angle_error
        else:
            self.get_logger().info('Goal reached!')
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publicar el comando de velocidad
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        """
        Detiene el robot publicando velocidades 0.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Robot stopped.')

def main(args=None):
    """
    Función principal para inicializar y ejecutar el nodo.
    """
    rclpy.init(args=args)

    controller = Controller()

    # Aquí puedes definir el objetivo a alcanzar
    goal_x = 5.0  # Ejemplo: coordenada x del objetivo
    goal_y = 0.5  # Ejemplo: coordenada y del objetivo

    try:
        rclpy.spin_once(controller)  # Procesar mensajes iniciales
        
        while rclpy.ok():
            if controller.robot_pose:
                controller.move_to_goal(goal_x, goal_y)
            rclpy.spin_once(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller node stopped manually.')
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
