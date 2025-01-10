import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
# Seguir testeando el distance threshold, mas info de ser necesario? Se a probado con 0.3 
# Se recopila informacion de la posicion del robot en el mapa. Se guarda informacion en un archivo .csv en los ejes x y 
class WaypointsRecorder(Node):
    def __init__(self):
        super().__init__('waypoints_recorder')
        self.subscription = self.create_subscription(Odometry,'/ego_racecar/odom',self.odom_callback,10)
        self.waypoints = []
        self.last_position = None
        self.distance_threshold = 0.1
        self.get_logger().info("Waypoints recorde node initialized.")
    
    def odom_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        x = round(x, 3)
        y = round(y, 3)
         
        # Si ya hay un waypoint guardado, calcular la distancia
        if self.last_position is not None:
            # Calcular la distancia entre el último waypoint y la posición actual
            distance_travelled = np.linalg.norm(np.array([x, y]) - np.array(self.last_position))
            
            # Si la distancia recorrida es mayor que el umbral, guardar el waypoint
            if distance_travelled >= self.distance_threshold:
                self.waypoints.append((x, y))
                self.last_position = (x, y)  # Actualizar la última posición
                self.get_logger().info(f"Recorded waypoint: x={x}, y={y} (Distance travelled: {distance_travelled:.2f}m)")
        else:
            # Si no hay último waypoint guardado, guardar el primero
            self.waypoints.append((x, y))
            self.last_position = (x, y)
            self.get_logger().info(f"Recorded initial waypoint: x={x}, y={y}")
    
    def save_waypoints_to_file(self, filename = "waypoints.csv"):
        with open(filename, 'w') as f:
            for waypoint in self.waypoints:
                f.write(f"{waypoint[0]:.3f},{waypoint[1]:.3f}\n")
        self.get_logger().info(f"Waypoints saved to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointsRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_waypoints_to_file()
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()