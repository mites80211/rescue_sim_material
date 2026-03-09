import numpy as np
import heapq
from collections import deque


def astar(grid, start, goal):
    rows, cols = grid.shape
    movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    open_nodes = []
    g_scores = {start: 0}
    f_scores = {start: heuristic(start, goal)}
    heapq.heappush(open_nodes, (f_scores[start], start))
    parents = {}

    while open_nodes:
        _, current_node = heapq.heappop(open_nodes)
        if current_node == goal:
            return construct_path(parents, current_node)
        for movement in movements:
            new_row = current_node[0] + movement[0]
            new_col = current_node[1] + movement[1]
            if 0 <= new_row < rows and 0 <= new_col < cols:
                neighbor = (new_row, new_col)
                if grid[neighbor]:
                    g_score = g_scores[current_node] + 1
                    if neighbor not in g_scores or g_score < g_scores[neighbor]:
                        g_scores[neighbor] = g_score
                        f_scores[neighbor] = g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_nodes, (f_scores[neighbor], neighbor))
                        parents[neighbor] = current_node
    return None


def heuristic(node, goal):
    return np.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)  # Euclidean distance


def bfs(grid, start, goal):
    rows, cols = grid.shape
    movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    queue = deque()
    visited = set()
    parents = {}

    queue.append(start)
    visited.add(start)

    while queue:
        current_node = queue.popleft()
        if current_node == goal:
            return construct_path(parents, current_node)
        for movement in movements:
            new_row = current_node[0] + movement[0]
            new_col = current_node[1] + movement[1]
            if 0 <= new_row < rows and 0 <= new_col < cols:
                neighbor = (new_row, new_col)
                if neighbor not in visited and grid[neighbor]:
                    queue.append(neighbor)
                    visited.add(neighbor)
                    parents[neighbor] = current_node
    return None


def construct_path(parents, goal):
    path = []
    current_node = goal
    while current_node in parents:
        path.append(current_node)
        current_node = parents[current_node]
    path.append(current_node)
    path.reverse()
    return path
