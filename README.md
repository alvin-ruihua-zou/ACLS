# Attractor-based Closed List Search: Sparsifying the Closed List for Efficient Memory-Constrained Planning

This is the code repository for the following paper accepted at IJCAI 2025.

Alvin Zou, Muhammad Suhail Saleem, Maxim Likhachev, "Attractor-based Closed List Search: Sparsifying the Closed List for Efficient Memory-Constrained Planning", IJCAI, 2025

## Repository Structure

- **Environments**:
  - `2dgrid.cpp` / `2dgrid.h`: 2D grid environment.
  - `3dgrid.cpp` / `3dgrid.h`: 3D grid environment.
  - `tiles.cpp` / `tiles.h`: Sliding Tiles environment.
  - `tower.cpp` / `tower.h`: Towers of Hanoi environment.
  - `environment.h`: Acts as the interface between the environment and the planner.

- **Pathfinding Algorithms**:
  - `astar.cpp` / `astar.h`: Implements the A* search algorithm.
  - `frontier.cpp` / `frontier.h`: Implements the Frontier A* search algorithm.
  - `smgs.cpp` / `smgs.h`: Implements the Sparse Memory Graph Search (SMGS) search algorithm.
  - `ACLS.cpp` / `ACLS.h`: Implements the Attractor-based Closed List Search (ACLS) search algorithm.
  - `LACLS.cpp` / `LACLS.h`: Implements the Lazy Attractor-based Closed List Search (LACLS) search algorithm.

- **Maps**:
  - `maps/`: Contains map files for the 2D grid environment.
  - `3d_maps/`: Contains map files for the 3D grid environment.
  - `15_tile_map.txt`: Contains maps for the Sliding Tiles environment.
  - `start_goal_pairs.txt`: Contains start and goal pairs for the 2D grid environment.

- **Test Files**:
  - `test_2d_grid.cpp`: Tests for the 2D grid environment.
  - `test_3d_grid.cpp`: Tests for the 3D grid environment.
  - `test_tiles_puzzle.cpp`: Tests for the Sliding Tiles puzzle environment.
  - `test_tower_puzzle.cpp`: Tests for the Towers of Hanoi puzzle environment.
  

## Building the Project

This project uses CMake for building. Follow these steps to build the project:

1. **Install CMake**:
   Ensure that CMake is installed on your system. You can download it from [cmake.org](https://cmake.org/).

2. **Create a Build Directory**:
   Open a terminal in the root of the repository and create a build directory:
   ```sh
   mkdir build
   cd build
   ```
3. **Generate Build Files**: 
   Run CMake to generate the build files:
   ```sh
   cmake ..
   ```
4. **Build the Project**:
   Compile the project using:
   ```sh
   cmake --build .
   ```
   Note: to run the tests for 2D grid and Sliding Tiles set the NUM_OF_ACTIONS variable in environment.h to 4. To run tests for 3D grid and Towers of Hanoi, set to 6.

## Running Tests
After building the project, you can run the test executables. Each test file corresponds to a specific environment:

```sh
./test_2d_grid
./test_3d_grid
./test_tiles_puzzle
./test_tower_puzzle
```

There are parameters such as the test file, heuristic and attractor distance function weight, and the option to record results that can be changed within each test file.