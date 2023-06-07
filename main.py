import random  # Import the random module for generating random numbers -ا
import math  # Import the math module for mathematical calculations
import heapq  # Import the heapq module for implementing the priority queue
import matplotlib.pyplot as plt  # Import the matplotlib module for visualization
import time  # Import the time module for adding delays

grid_size = 10  # Set the size of the grid
start_point = (0, 0)  # Initialize the start point
end_point = (0, 0)  # Initialize the end point
obstacles = []  # Initialize the list of obstacles

# Function to create an empty grid
def create_grid(size):
    return [['.'] * size for _ in range(size)]

# Function to add random obstacles to the grid
def add_obstacles(grid, num_obstacles):
    for _ in range(num_obstacles):
        # Generate a random obstacle position
        obstacle = (random.randint(0, grid_size - 1), random.randint(0, grid_size - 1))
        # Ensure the obstacle is not on the start point, end point, or existing obstacles
        while obstacle == start_point or obstacle == end_point or obstacle in obstacles:
            obstacle = (random.randint(0, grid_size - 1), random.randint(0, grid_size - 1))
        # Add the obstacle to the grid and obstacles list
        grid[obstacle[0]][obstacle[1]] = '#'
        obstacles.append(obstacle)

# Function to calculate the distance between two points
# we need it to find heuristic and cost
def calculate_distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    # Calculate Euclidean distance between two points
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Function to find the neighbors of a point in the grid
def find_neighbors(point, grid):
    x, y = point
    neighbors = [] # تنشأ مصفوفه فاضه تخزن فيها
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            # Check if the neighbor is within grid boundaries and not an obstacle
            if 0 <= nx < grid_size and 0 <= ny < grid_size and grid[nx][ny] != '#':
                neighbors.append((nx, ny))
    return neighbors

# Function to calculate the heuristic value of a point
def heuristic(point, goal):
    return calculate_distance(point, goal)

# Function to reconstruct the path from the start point to the current point
def reconstruct_path(came_from, current):
    path = [current]
    cost = 0
    while current in came_from:
        prev = current
        current = came_from[current]
        # Calculate the distance between consecutive points and update the cost
        cost += calculate_distance(current, prev)
        path.append(current)
    # Reverse the path to get it from start to end point .
    return path[::-1], cost

# Function to find the optimal path using A* algorithm
def find_path(grid, start, end):
    open_set = []  # Initialize the open set as an empty list  --
    closed_set = set()  # Initialize the closed set as an empty set --
    came_from = {}  # Initialize the came_from dictionary to store the path -
    g_score = {start: 0}  # Initialize the g_score dictionary to store the cost from start to each point
    f_score = {start: heuristic(start, end)}  # Initialize the f_score dictionary to store the total cost from start to each point

    heapq.heappush(open_set, (f_score[start], start))  # Add the start point to the open set with its f_score value
    while open_set:
        _, current = heapq.heappop(open_set)  # Get the point with the lowest f_score from the open set --


        if current == end:
            # Reconstruct the path and return it along with the cost
            return reconstruct_path(came_from, current)

        closed_set.add(current)  # Add the current point to the closed set

        neighbors = find_neighbors(current, grid)  # Find the neighbors of the current point
        for neighbor in neighbors:
            if neighbor in closed_set:
                continue

            # Calculate the tentative g_score based on the movement direction
            tentative_g_score = g_score[current] + 1 if neighbor[0] == current[0] or neighbor[1] == current[1] else \
                g_score[current] + math.sqrt(2)

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                # Update the path and g_score for the neighbor
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                if neighbor not in open_set:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # Update visualization
        update_visualization(grid, came_from, current)
        time.sleep(0.001)

    # If no path is found
    return None, None

# Function to update the visualization of the grid
def update_visualization(grid, came_from, current):
    plt.clf()  # Clear the current figure
    plt.xlim(0, grid_size)  # Set the x-axis limits
    plt.ylim(0, grid_size)  # Set the y-axis limits

    for x in range(grid_size):
        for y in range(grid_size):
            if grid[x][y] == '#':
                # Draw obstacle cell as black
                plt.fill([y, y + 1, y + 1, y], [grid_size - x - 1, grid_size - x - 1, grid_size - x, grid_size - x], 'black')
            elif grid[x][y] == 'S':
                # Draw start point cell as green
                plt.fill([y, y + 1, y + 1, y], [grid_size - x - 1, grid_size - x - 1, grid_size - x, grid_size - x], 'green')
            elif grid[x][y] == 'E':
                # Draw end point cell as red
                plt.fill([y, y + 1, y + 1, y], [grid_size - x - 1, grid_size - x - 1, grid_size - x, grid_size - x], 'red')
            elif grid[x][y] == 'X':
                # Draw path cell as blue
                plt.fill([y, y + 1, y + 1, y], [grid_size - x - 1, grid_size - x - 1, grid_size - x, grid_size - x], 'blue')

    # Display explored nodes
    for point in came_from:
        if point != start_point and point != end_point:
            plt.fill([point[1], point[1] + 1, point[1] + 1, point[1]],
                     [grid_size - point[0] - 1, grid_size - point[0] - 1, grid_size - point[0], grid_size - point[0]],
                     'lightgray')

    # Display current path being considered
    if current != start_point and current != end_point:
        plt.fill([current[1], current[1] + 1, current[1] + 1, current[1]],
                 [grid_size - current[0] - 1, grid_size - current[0] - 1, grid_size - current[0], grid_size - current[0]],
                 'yellow')

    plt.draw()  # Draw the updated grid
    plt.pause(0.001)  # Add a small delay for visualization

# Create grid and randomize start/end points
grid = create_grid(grid_size)  # Create an empty grid

start_point = (random.randint(0, grid_size - 1), random.randint(0, grid_size - 1))  # Randomize the start point
grid[start_point[0]][start_point[1]] = 'S'  # Mark the start point on the grid

end_point = (random.randint(0, grid_size - 1), random.randint(0, grid_size - 1))  # Randomize the end point
while end_point == start_point:
    end_point = (random.randint(0, grid_size - 1), random.randint(0, grid_size - 1))
grid[end_point[0]][end_point[1]] = 'E'  # Mark the end point on the grid

add_obstacles(grid, 40)  # Add 40 random obstacles to the grid

# Create a figure and axis for visualization
fig, ax = plt.subplots()

print("Grid with random starting and ending points, and 10 random obstacles:")
for row in grid:
    print(' '.join(row))  # Print the grid with start, end, and obstacles marked

# Find the path from start to end point
path, path_cost = find_path(grid, start_point, end_point)

if path:
    print("\nPath from start to end point:")
    for i, point in enumerate(path):
        x, y = point
        grid[x][y] = 'X'  # Mark the path cells on the grid
        update_visualization(grid, {}, (0, 0))  # Update the visualization
        print("Step", i + 1)
        for row in grid:
            print(' '.join(row))  # Print the grid at each step
        print("Path:", path)  # Print the final path
        print("Path cost:", path_cost)  # Print the cost of the path
        time.sleep(0.01)  # Add a small delay between steps
else:
    print("\nNo path found from start to end point.")

plt.show()  # Show the final visualization
