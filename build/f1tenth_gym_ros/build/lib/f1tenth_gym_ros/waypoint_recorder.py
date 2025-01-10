import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class WaypointsRecorder(Node):
    def __init__(self):
        super().__init__('waypoints_recorder')
        self.subscription = self.create_subscription(Odometry,'/ego_racecar/odom',self.odom_callback,10)
        self.waypoints = []
        self.get_logger().info("Waypoints recorde node initialized.")
    
    def odom_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.waypoints.append((x,y))
        self.get_logger().info(f"Recorded waypoint: x={x}, y={y}")
    
    def save_waypoints_to_file(self, filename = "waypoints.csv"):
        with open(filename, 'w') as f:
            for waypoint in self.waypoints:
                f.write(f"{waypoint[0], {waypoint[1]}}\n")
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