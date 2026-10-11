# robot_search
## Warehouse Delivery Robot - Search Algorithms
Intelligent Systems, Task 1 - Alexandria University

A robot moves from the start cell `S` to the goal cell `G` on a grid map (Up, Down, Left, Right).
Normal floor `.` costs 1, rough floor `~` costs 3, and a wall `#` cannot be entered.

## Algorithms
- **UCS** (Uniform-Cost Search) - uninformed, finds the cheapest path.
- **A\*** with Manhattan distance heuristic - informed, finds the cheapest path while expanding fewer nodes.

The program is written from scratch (no path-finding libraries) and only uses `heapq` and `time` from the
Python standard library. No extra packages are needed.

## Problem Formulation
The problem is a cheapest-path search on a 2D grid. Each cell is one of the following:
- `S`: start cell (cost not counted).
- `G`: goal cell (cost of entering = 1).
- `.`: normal floor (cost of entering = 1).
- `~`: rough floor (cost of entering = 3).
- `#`: wall (cannot be entered).

The robot moves Up, Down, Left or Right, one cell at a time. The goal is to find a path from `S` to `G` that
minimizes the total cost of the cells entered. If no path exists, the program prints "No path found".
Coordinates are written as `(row, column)`, where row 0 is the top row.

## Algorithm Descriptions
**UCS (Uniform-Cost Search)**
UCS is an uninformed search algorithm. It expands the node with the lowest accumulated path cost g(n), using a
priority queue (min-heap) ordered by g(n). Because it always expands the cheapest node first, it is guaranteed
to find the cheapest path. It does not use any information about where the goal is.

**A\* (A-Star)**
A* is an informed search algorithm. It expands the node with the lowest f(n) = g(n) + h(n), where g(n) is the
cost from the start to n and h(n) is a heuristic estimate of the cost from n to the goal. A* finds the cheapest
path if the heuristic is admissible. It usually expands fewer nodes than UCS because the heuristic guides the
search toward the goal.

## Heuristic
The heuristic used in A* is the Manhattan distance between a cell (r1, c1) and the goal (r2, c2):

    h(n) = |r1 - r2| + |c1 - c2|

It is **admissible** because the cheapest possible move costs 1 and the robot moves in 4 directions only (no
diagonals), so the Manhattan distance never overestimates the true remaining cost. It is also **consistent**,
so A* with a closed set returns an optimal path.

## Code structure
- `robot_search_combined.py` - the full program in one file (**run this one**).
- `team_parts/` - the same code split into three parts, one per team member:
  - `part1.py` - maps, helper functions, Manhattan heuristic, A*
  - `part2_ucs.py` - UCS
  - `part3.py` - running the algorithms, printing the paths, results table and comparison

## How to run
1. Install Python 3 (or Anaconda). No extra packages are needed.
2. Open a terminal (or Anaconda Prompt) in the folder of the project and run:

       python robot_search_combined.py

The program runs both algorithms on MAP_A, MAP_B and MAP_C, then prints for each: the path, path length, path
cost, nodes expanded, maximum frontier size, run time, the map with the path marked by `*`, a final results
table, and a comparison between UCS and A*.

To run the three parts as separate files instead, keep `part1.py`, `part2_ucs.py` and `part3.py` in the same
folder and run:

    python part3.py

## How to change the map
The maps `MAP_A`, `MAP_B` and `MAP_C` are defined at the top of the code (Part 1), and `GRID` holds the
default map. The program runs the maps listed in the `maps` list inside `main()` (Part 3). To change them, edit
that list, for example to run only MAP_B:

    maps = [("MAP_B", MAP_B)]

You can also write your own map as a list of strings using `S` (start), `G` (goal), `.`, `~` and `#`, define it
next to the other maps, and add it to the `maps` list.

## Results summary
| Map | Algorithm | Path length | Path cost | Nodes expanded | Max frontier | Run time (ms) |
|---|---|---|---|---|---|---|
| MAP_A | UCS | 11 | 15 | 28 | 4 | 0.070 |
| MAP_A | A* | 11 | 15 | 21 | 5 | 0.059 |
| MAP_B | UCS | 12 | 12 | 30 | 4 | 0.149 |
| MAP_B | A* | 12 | 12 | 15 | 7 | 0.107 |
| MAP_C | UCS | None | None (No path found) | 13 | 3 | 0.057 |
| MAP_C | A* | None | None (No path found) | 13 | 3 | 0.030 |

Run times are below 1 ms on these maps and change from one run to another (they depend on the computer and
on the run), so they are only a rough value. The comparison below is based on the nodes expanded.

MAP_B contains the trap: the shortest path (8 moves) costs 22, while the cheapest path (12 moves) costs 12.
Both UCS and A* choose the cheapest path.

## Comparison: UCS vs A*
| Map | UCS expanded | A* expanded | Result |
|---|---|---|---|
| MAP_A | 28 | 21 | A* expanded 7 fewer nodes, same cost (15) |
| MAP_B | 30 | 15 | A* expanded half the nodes, same cost (12) |
| MAP_C | 13 | 13 | No path exists, so the heuristic cannot help |

Both algorithms find the same cheapest path. A* expands fewer nodes on MAP_A and MAP_B because the Manhattan
heuristic guides the search toward the goal, at the price of a slightly larger frontier (7 vs 4 on MAP_B).

## Conclusion
UCS and A* both guarantee the cheapest path. A* is more efficient because it expands fewer nodes while still
producing the same optimal cost, so it is the preferred choice for a warehouse delivery robot. The trap in
MAP_B confirms that the shortest path is not always the cheapest, and both algorithms handle it correctly by
minimizing cost rather than the number of moves.

## Team
- مرام ابراهيم خميس عبدالله - 2403258504
- منة احمد سيد احمد - 2403252144
- مريم محمد عبدالباري محمد - 2403251016
