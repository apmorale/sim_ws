# astar_planner_node.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
import numpy as np
import heapq

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
        self.map_data = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin = None
        
        # Suscribirse al mapa
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Publicar el camino
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.get_logger().info("A* Planner Node initialized")

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = msg.info.origin.position

        # Ahora que tenemos el mapa, iniciar el cálculo del camino
        start = (10, 10)  # ejemplo de inicio
        goal = (100, 100)  # ejemplo de objetivo
        self.get_logger().info(f"Start: {start}, Goal: {goal}")

        path = self.a_star(start, goal)
        self.publish_path(path)

    def a_star(self, start, goal):
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def neighbors(node):
            x, y = node
            neighbor_coords = [
                (x+1, y), (x-1, y), (x, y+1), (x, y-1)
            ]
            valid_neighbors = []
            for nx, ny in neighbor_coords:
                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                    if self.map_data[ny, nx] != 100:  # 100 es obstáculo
                        valid_neighbors.append((nx, ny))
            return valid_neighbors

        # A* implementation
        open_list = []
        heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start))
        came_from = {}
        g_score = {start: 0}

        while open_list:
            _, current_g, current = heapq.heappop(open_list)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # Revertir el camino

            for neighbor in neighbors(current):
                tentative_g = current_g + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    heapq.heappush(open_list, (tentative_g + heuristic(neighbor, goal), tentative_g, neighbor))

        return []  # No path found

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for p in path:
            pose = PoseStamped()
            pose.pose.position.x = p[0] * self.map_resolution + self.map_origin.x
            pose.pose.position.y = p[1] * self.map_resolution + self.map_origin.y
            pose.pose.orientation.w = 1.0  # Orientación arbitraria
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info(f"Path published with {len(path)} points")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
