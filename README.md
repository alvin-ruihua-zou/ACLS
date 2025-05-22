# Attractor-based Closed List Search: Sparsifying the Closed List for Efficient Memory-Constrained Planning

This is the code repository for the following paper accepted at IJCAI 2025.

Alvin Zou, Muhammad Suhail Saleem, Maxim Likhachev, "Attractor-based Closed List Search: Sparsifying the Closed List for Efficient Memory-Constrained Planning", IJCAI, 2025

## Repository Structure

- **Environments**:
  - `2dgrid.cpp` / `2dgrid.h`: 2D grid environment.
  - `3dgrid.cpp` / `3dgrid.h`: 3D grid environment.
  - `tiles.cpp` / `tiles.h`: Sliding Tiles environment.
  - `tower.cpp` / `tower.h`: Towers of Hanoi environment.

- **Pathfinding Algorithms**:
  - `astar.cpp` / `astar.h`: Implements the A* search algorithm.
  - `frontier.cpp` / `frontier.h`: Implements the Frontier A* search algorithm.
  - `smgs.cpp` / `smgs.h`: Implements the Sparse Memory Graph Search (SMGS) search algorithm.
  - `ACLS.cpp` / `ACLS.h`: Implements the Attractor-based Closed List Search (ACLS) search algorithm.
  - `LACLS.cpp` / `LACLS.h`: Implements the Lazy Attractor-based Closed List Search (LACLS) search algorithm.

- **Environment**:
  - `environment.h`: Defines the environment for running simulations and algorithms.


- **Test Files**:
  - `test_2d_grid.cpp`: Tests for the 2D grid environment.
  - `test_3d_grid.cpp`: Tests for the 3D grid environment.
  - `test_tiles_puzzle.cpp`: Tests for the Sliding Tiles puzzle environment.
  - `test_tower_puzzle.cpp`: Tests for the Towers of Hanoi puzzle environment.
  

## Running Tests

To run the test files, follow these steps:

1. **Compile the Test Files**:
   Use a C++ compiler (e.g., `g++`) to compile the test files.

   2D environment:
   ```sh
   g++ -o 2dgrid.out 2dgrid.h 2dgrid.cpp compares.h compares.cpp smgs.h smgs.cpp ACLS.h ACLS.cpp LACLS.h LACLS.cpp frontier.h frontier.cpp astar.h astar.cpp readMap.h readMap.cpp test_2d_grid.cpp
   ```
   
   3D environment:
   ```sh
   g++ -o 3dgrid.out 3dgrid.h 3dgrid.cpp compares.h compares.cpp ACLS.h ACLS.cpp LACLS.h LACLS.cpp frontier.h frontier.cpp astar.h astar.cpp smgs.h smgs.cpp readMap.h readMap.cpp test_3d_grid.cpp
   ```

   Sliding Tiles environment:
   ```sh
   g++ -o tiles.out tiles.h tiles.cpp compares.h compares.cpp smgs.h smgs.cpp ACLS.h ACLS.cpp LACLS.h LACLS.cpp frontier.h frontier.cpp astar.h astar.cpp test_tiles_puzzle.cpp
   ```

   Towers of Hanoi environment:
   ```sh
   g++ -o tower.out tower.h tower.cpp compares.h compares.cpp smgs.h smgs.cpp ACLS.h ACLS.cpp LACLS.h LACLS.cpp frontier.h frontier.cpp astar.h astar.cpp test_tower_puzzle.cpp
   ```

2. Run the complied executable:
    ```sh
    ./2dgrid.out
    ./3dgrid.out
    ./tiles.out
    ./tower.out
    ```

There are parameters such as the test file, heuristic and attractor distance function weight, and the option to record results that can be changed within each test file.