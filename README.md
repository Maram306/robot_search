# robot_search
# Warehouse Delivery Robot - Search Algorithms

Intelligent Systems, Task 1 - Alexandria University

A robot moves from the start cell S to the goal cell G on a grid map
(Up, Down, Left, Right). Normal floor `.` costs 1, rough floor `~` costs 3,
and a wall `#` cannot be entered.

## Algorithms
- **UCS** (Uniform-Cost Search) - uninformed, finds the cheapest path.
- **A\*** with Manhattan distance heuristic - informed, finds the cheapest path
  while expanding fewer nodes.

No path-finding libraries are used (only `heapq` and `time` from the standard library).

## How to run
1. Install Python 3 (or Anaconda).
2. Open `robot_search.ipynb` in Jupyter Notebook / JupyterLab.
3. Click **Run All**.

The program runs both algorithms on MAP_A, MAP_B and MAP_C, then prints for each:
the path, path cost, nodes expanded, the map with the path marked by `*`,
and a final results table (path length, cost, nodes expanded, max frontier, run time).

## How to change the map
Edit the `GRID` variable at the top of the code, for example:

    GRID = MAP_B

You can also write your own map as a list of strings using
`S` (start), `G` (goal), `.`, `~` and `#`, and assign it to `GRID`.

## Results summary
| Map | Algorithm | Cost | Nodes expanded |
|---|---|---|---|
| MAP_A | UCS | 15 | 28 |
| MAP_A | A* | 15 | 21 |
| MAP_B | UCS | 12 | 30 |
| MAP_B | A* | 12 | 15 |
| MAP_C | UCS / A* | No path found | 13 |
