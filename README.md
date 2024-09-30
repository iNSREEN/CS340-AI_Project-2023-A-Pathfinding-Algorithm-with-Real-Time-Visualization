# A* Pathfinding Algorithm Visualization

This project implements the A* pathfinding algorithm to find the shortest path on a 2D grid with randomly placed obstacles. The grid is visualized using Matplotlib, providing a dynamic view of the algorithm's execution as it explores the grid and finds the optimal path from a start point to an end point.

## Features

- **Randomized Grid**: A 10x10 grid with random start and end points.
- **Obstacles**: 40 random obstacles placed on the grid.
- **A* Algorithm**: Implements the A* pathfinding algorithm to calculate the shortest path.
- **Visualization**: Real-time visualization of the algorithm's progress, showing explored nodes, current path, and final path.

## Prerequisites

Before running the project, ensure you have the following Python libraries installed:

- `matplotlib`: For visualization
- `heapq`: For implementing the priority queue
- `math`: For mathematical calculations
- `random`: For generating random numbers

You can install the required libraries using pip:

```bash
pip install matplotlib
```

2. **View the Output**: A Matplotlib window will open, showing the grid and the pathfinding process in real-time.

## Code Overview

- **create_grid(size)**: Creates an empty grid of the specified size.
- **add_obstacles(grid, num_obstacles)**: Randomly places obstacles in the grid.
- **calculate_distance(p1, p2)**: Computes the Euclidean distance between two points.
- **find_neighbors(point, grid)**: Finds valid neighboring points for a given point.
- **heuristic(point, goal)**: Calculates the heuristic value for A*.
- **reconstruct_path(came_from, current)**: Reconstructs the path from the start to the current point.
- **find_path(grid, start, end)**: Implements the A* algorithm to find the optimal path.
- **update_visualization(grid, came_from, current)**: Updates the grid visualization during the algorithm's execution.

## Example Output

The program outputs the grid representation with the following symbols:
- `S`: Start point
- `E`: End point
- `#`: Obstacles
- `X`: Path taken
- `.`: Empty cells

As the algorithm runs, the explored cells are shown in light gray, the current point in yellow, and the path in blue.
