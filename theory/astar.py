# Code implementation for section 3.3 of report 
# Path Planning with A* Search Algorithm
import math
import matplotlib.pyplot as plt

class Node:
    def __init__(self, position, parent=None):
        self.position = position  # (x, y) coordinates on 2D grid map 
        self.parent = parent  
        self.g = 0  # Cost from start node to this node
        self.h = 0  # Heuristic cost to the goal
        self.f = 0  # Total cost (g + h)

    def __eq__(self, other):
        return self.position == other.position

def heuristic(a, b):
    # Use Euclidean distance as the heuristic (note, heuristic is subjective for each application, refer to FYP report for more details)
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def get_neighbors(node, grid):
    neighbors = []
    directions = [
        (-1, 0),  # Up
        (1, 0),   # Down
        (0, -1),  # Left
        (0, 1),   # Right
        (-1, -1), # Top-left (diagonal)
        (-1, 1),  # Top-right (diagonal)
        (1, -1),  # Bottom-left (diagonal)
        (1, 1)    # Bottom-right (diagonal)
    ]

    for d in directions:
        neighbor_pos = (node.position[0] + d[0], node.position[1] + d[1])
        # Ensure that neighbour can be navigated to 
        if 0 <= neighbor_pos[0] < len(grid) and 0 <= neighbor_pos[1] < len(grid[0]):
            if grid[neighbor_pos[0]][neighbor_pos[1]] == 0:  # 0 indicates walkable space
                neighbors.append((Node(neighbor_pos), d))
    return neighbors

def reconstruct_path(current_node):
    path = []
    while current_node is not None:
        path.append(current_node.position)
        current_node = current_node.parent
    return path[::-1]  # Return reversed path

def A_Star_Search(grid, start, goal):
    # Initialize the start and goal nodes
    start_node = Node(start)
    goal_node = Node(goal)

    # Open and closed lists
    open_set = [start_node]
    closed_set = []

    while open_set:
        # Find the node with the lowest f in the open set (algorithm optimize by finding lowest cost )
        current_node = open_set[0]
        for node in open_set:
            if node.f < current_node.f:
                current_node = node

        open_set.remove(current_node)
        closed_set.append(current_node)

        print(current_node.position)

        # Check if at goal state
        if current_node == goal_node:
            return reconstruct_path(current_node)

        # Get neighbour nodes
        neighbors = get_neighbors(current_node, grid)
        for neighbor, direction in neighbors:
            if neighbor in closed_set:
                continue  # Skip already evaluated neighbors

            # Calculate the cost to move to this neighbor
            if direction[0] != 0 and direction[1] != 0:  # Diagonal movement
                step_cost = 2
            else:  # Straight movement
                step_cost = 1

            tentative_h = current_node.g + step_cost + heuristic(current_node.position, goal_node.position)

            if neighbor in open_set:
                if tentative_h < neighbor.h:
                    neighbor.h = tentative_h
                    neighbor.parent = current_node
            else:
                neighbor.g = current_node.g + step_cost
                neighbor.h = heuristic(neighbor.position, goal_node.position)
                neighbor.f = neighbor.g + neighbor.h
                neighbor.parent = current_node
                open_set.append(neighbor)

    # No goal found, return none 
    return None

def visualize_grid(grid, path, start, goal):
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap='binary', origin='upper')
    plt.grid(True, which='both', color='gray', linewidth=0.5)

    # Mark the path
    if path:
        for position in path:
            plt.scatter(position[1], position[0], color='blue', s=100)  # Path in blue

    # Mark the start and goal
    plt.scatter(start[1], start[0], color='green', s=200, label='Start')  # Start in green
    plt.scatter(goal[1], goal[0], color='red', s=200, label='Goal')  # Goal in red

    # Add legend
    plt.legend(loc='upper right')
    plt.show()

# Example usage
if __name__ == "__main__":
    # 0 represents walkable space, 1 represents obstacles (edit grids here to show different results)
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 1, 1, 1, 0],
        [0, 1, 0, 0, 0]
    ]

    start = (0, 0)
    goal = (4, 4)

    path = A_Star_Search(grid, start, goal)
    if path:
        print("Path found:", path)
        visualize_grid(grid, path, start, goal)
        
    else:
        print("No path found.")
