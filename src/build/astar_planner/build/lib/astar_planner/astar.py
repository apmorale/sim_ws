import heapq
import numpy as np

class AStar:
    def __init__(self, grid):
        """
        Constructor para inicializar la matriz de ocupación.
        :param grid: np.array, un mapa donde 0 representa espacio libre y valores mayores a 0 representan obstáculos.
        """
        self.grid = grid
        self.rows, self.cols = grid.shape
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0),  # Movimientos cardinales
                           (1, 1), (1, -1), (-1, 1), (-1, -1)]  # Movimientos diagonales

    def is_valid(self, x, y):
        """
        Verifica si una celda está dentro del mapa y no es un obstáculo.
        """
        return 0 <= x < self.rows and 0 <= y < self.cols and self.grid[x, y] == 0

    def heuristic(self, a, b):
        """
        Calcula la heurística (distancia Manhattan o Euclidiana).
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan
        # return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)  # Euclidiana

    def astar(self, start, goal, treshold=50):
        """
        Encuentra un camino usando A*.
        :param start: tuple (x, y), posición inicial.
        :param goal: tuple (x, y), posición objetivo.
        :param treshold: int, valor límite para identificar obstáculos en la matriz.
        :return: List de coordenadas que forman el camino, o None si no hay camino.
        """
        if self.grid[start[0], start[1]] >= treshold or self.grid[goal[0], goal[1]] >= treshold:
            return None  # No hay camino si el inicio o el objetivo están en obstáculos

        # Inicialización
        open_set = []
        heapq.heappush(open_set, (0, start))  # (f_score, posición)
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            # Verificar si hemos alcanzado el objetivo
            if current == goal:
                return self.reconstruct_path(came_from, current)

            # Explorar vecinos
            for dx, dy in self.directions:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue

                tentative_g_score = g_score[current] + 1  # Asumiendo peso uniforme

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # Actualizar costos y registrar el camino
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No hay camino

    def reconstruct_path(self, came_from, current):
        """
        Reconstruye el camino desde el nodo objetivo al nodo inicial.
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
