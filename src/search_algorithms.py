import random
import heapq
from collections import deque

# Directions: Up, Down, Left, Right
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# Random movement algorithm (limited moves)
def random_move(start, goal, obstacles, rows, cols):
    path = []
    current = start

    for _ in range(1000):  # Limit to 1000 moves to avoid infinite loops
        direction = random.choice(DIRECTIONS)  # Choose a random direction
        new_pos = (current[0] + direction[0], current[1] + direction[1])  # Calculate new position

        # Check if new position is within grid bounds and not an obstacle
        if (
            0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and
            new_pos not in obstacles
        ):
            path.append(direction)  # Record the move
            current = new_pos      # Update current position

        if current == goal:  # If the goal is reached, return the path
            return path

    return []  # If 1000 moves are exhausted without reaching the goal, return an empty path

# Breadth First Search (BFS) Algorithm
def bfs(start, goal, obstacles, rows, cols):
    queue = deque([(start, [])])  # Initialize queue with starting position and empty path
    visited = set([start])        # Mark start as visited

    while queue:
        (x, y), path = queue.popleft()  # Dequeue the next node and its path

        if (x, y) == goal:  # Check if we have reached the goal
            return path

        # Explore all possible directions
        for dx, dy in DIRECTIONS:
            new_x, new_y = x + dx, y + dy

            # If the new position is valid, not an obstacle, and not visited
            if (0 <= new_x < rows and 0 <= new_y < cols and 
                (new_x, new_y) not in obstacles and (new_x, new_y) not in visited):
                queue.append(((new_x, new_y), path + [(dx, dy)]))  # Enqueue new node with updated path
                visited.add((new_x, new_y))  # Mark new node as visited

    return []  # Return empty path if goal is not reached

# Depth First Search (DFS) Algorithm
def dfs(start, goal, obstacles, rows, cols):
    stack = [(start, [])]  # Use a stack for DFS, starting with the start position and empty path
    visited = set([start])  # Track visited nodes to prevent cycles

    while stack:
        current, path = stack.pop()  # Pop the last element (LIFO order)

        if current == goal:  # Check if the goal is reached
            return path

        # Explore neighbors
        for dx, dy in DIRECTIONS:
            new_pos = (current[0] + dx, current[1] + dy)
            # If the neighbor is valid, not an obstacle, and not visited
            if (0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and
                new_pos not in obstacles and new_pos not in visited):
                visited.add(new_pos)  # Mark as visited
                stack.append((new_pos, path + [(dx, dy)]))  # Add neighbor to the stack with updated path
    return []  # Return empty path if no solution is found

# Iterative Deepening Search (IDS) Algorithm
def ids(start, goal, obstacles, rows, cols):
    def dls(node, depth, path, visited):
        # Depth-Limited Search helper function
        if node == goal:  # Base case: goal reached
            return path
        if depth == 0:  # Reached depth limit without success
            return None
        for dx, dy in DIRECTIONS:
            new_node = (node[0] + dx, node[1] + dy)
            # If new_node is valid, not an obstacle, and not visited in current path
            if (0 <= new_node[0] < rows and 0 <= new_node[1] < cols and
                new_node not in obstacles and new_node not in visited):
                visited.add(new_node)  # Mark the node as visited for this path
                result = dls(new_node, depth - 1, path + [(dx, dy)], visited)  # Recurse with decreased depth
                if result is not None:  # If a solution is found in recursion, return it
                    return result
                visited.remove(new_node)  # Backtrack if not found
        return None

    # Set maximum depth to total number of cells as worst-case
    max_depth = rows * cols
    for depth in range(max_depth):
        visited = set([start])  # Reset visited set for each iteration
        result = dls(start, depth, [], visited)
        if result is not None:  # If path is found within current depth limit, return it
            return result
    return []  # Return empty path if no solution is found within max_depth

# Uniform Cost Search (UCS) Algorithm
def ucs(start, goal, obstacles, rows, cols):
    pq = []  # Priority queue to store (cost, node, path)
    heapq.heappush(pq, (0, start, []))  # Initialize with start node, cost 0, and empty path
    visited = {start: 0}  # Dictionary to track the minimum cost to reach a node

    while pq:
        cost, node, path = heapq.heappop(pq)  # Pop the node with the smallest cost
        if node == goal:  # Check if the goal is reached
            return path
        for dx, dy in DIRECTIONS:
            new_node = (node[0] + dx, node[1] + dy)
            # If new_node is valid and not an obstacle
            if (0 <= new_node[0] < rows and 0 <= new_node[1] < cols and 
                new_node not in obstacles):
                new_cost = cost + 1  # Each move costs 1
                # If new_node has not been visited or found a cheaper cost path
                if new_node not in visited or new_cost < visited[new_node]:
                    visited[new_node] = new_cost  # Update cost for new_node
                    heapq.heappush(pq, (new_cost, new_node, path + [(dx, dy)]))  # Push new_node into the queue
    return []  # Return empty path if goal is not reached

# Greedy Best First Search Algorithm
def greedy_bfs(start, goal, obstacles, rows, cols):
    def heuristic(a, b):
        # Manhattan distance: estimated cost from node a to b
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    pq = []  # Priority queue to store (heuristic value, node, path)
    heapq.heappush(pq, (heuristic(start, goal), start, []))  # Start with the start node
    visited = set([start])  # Track visited nodes

    while pq:
        _, node, path = heapq.heappop(pq)  # Pop the node with lowest heuristic value
        if node == goal:  # Check if goal is reached
            return path
        for dx, dy in DIRECTIONS:
            new_node = (node[0] + dx, node[1] + dy)
            # If new_node is valid, not an obstacle, and not visited
            if (0 <= new_node[0] < rows and 0 <= new_node[1] < cols and 
                new_node not in obstacles and new_node not in visited):
                visited.add(new_node)  # Mark as visited
                # Push new_node with its heuristic cost into the priority queue
                heapq.heappush(pq, (heuristic(new_node, goal), new_node, path + [(dx, dy)]))
    return []  # Return empty path if goal is not reached

# A* Search Algorithm
def astar(start, goal, obstacles, rows, cols):
    def heuristic(a, b):
        # Manhattan distance heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    pq = []  # Priority queue to store (f, cost, node, path)
    # f = cost so far + heuristic estimate to goal
    heapq.heappush(pq, (heuristic(start, goal), 0, start, []))
    visited = {start: 0}  # Track the best cost to each node

    while pq:
        f, cost, node, path = heapq.heappop(pq)  # Pop the node with smallest f value
        if node == goal:  # Check if goal is reached
            return path
        for dx, dy in DIRECTIONS:
            new_node = (node[0] + dx, node[1] + dy)
            # Ensure new_node is valid and not an obstacle
            if (0 <= new_node[0] < rows and 0 <= new_node[1] < cols and 
                new_node not in obstacles):
                new_cost = cost + 1  # Each move costs 1
                # If this path to new_node is the best so far, or new_node is unvisited
                if new_node not in visited or new_cost < visited[new_node]:
                    visited[new_node] = new_cost  # Update cost for new_node
                    new_f = new_cost + heuristic(new_node, goal)  # Calculate new f value
                    # Push new_node with updated cost and path into the queue
                    heapq.heappush(pq, (new_f, new_cost, new_node, path + [(dx, dy)]))
    return []  # Return empty path if goal is not reached