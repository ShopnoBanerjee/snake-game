import random
from collections import deque

# Directions (Up, Down, Left, Right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# Random movement algorithm (limited moves)
def random_move(start, goal, obstacles, rows, cols):
    path = []
    current = start

    for _ in range(1000):  # Limit to 1000 moves
        direction = random.choice(DIRECTIONS)
        new_pos = (current[0] + direction[0], current[1] + direction[1])

        if (
            0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and
            new_pos not in obstacles
        ):
            path.append(direction)
            current = new_pos

        if current == goal:
            return path

    return []  # If it takes too long, return empty path


# Write your code below this only

# Breadth First Search (BFS) Algorithm
def bfs(start, goal, obstacles, rows, cols):
    pass

# Depth First Search (DFS) Algorithm
def dfs(start, goal, obstacles, rows, cols):
    pass

# Iterative Deepening Search (IDS Algorithm
def ids(start, goal, obstacles, rows, cols):
    pass

# Uniform Cost Search (UCS) Algorithm
def ucs(start, goal, obstacles, rows, cols):
    pass

# Greedy Best First Search Algorithm
def greedy_bfs(start, goal, obstacles, rows, cols):
    pass

# A* Search Algorithm
def astar(start, goal, obstacles, rows, cols):
    pass
