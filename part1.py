"""
PART 1 (Member 1): maps, helper functions, Manhattan heuristic, and A*.
Run this part FIRST.

Warehouse Delivery Robot
Algorithms implemented from scratch:
  1. UCS  - uninformed, minimises total path cost
  2. A*   - informed, f(n) = g(n) + h(n), h = Manhattan distance

HOW TO RUN: run Part 1, then Part 2, then Part 3.
HOW TO CHANGE THE MAP: edit GRID below (e.g. GRID = MAP_B), then run
    run_map("GRID", GRID, [])
in a new cell. Or assign your own list of strings (S, G, ., ~, #) to GRID.
"""
import heapq
import time

# Maps  (S start, G goal, . cost 1, ~ cost 3, # wall)
MAP_A = [
    "S..#....",
    "##.#.##.",
    "....~~..",
    ".####.#.",
    "...~..#G",
]
MAP_B = [
    "S~~~~~~~G",
    ".#######.",
    ".........",
    ".#.#.#.#.",
    ".........",
]
MAP_C = [
    "S..#....",
    ".#.#.##.",
    "...#..#.",
    ".###.##.",
    "....#..G",
]

GRID = MAP_A          # <<< change this to MAP_B / MAP_C / your own map

MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]


# ---------------------------- Helpers ----------------------------
def find_cell(grid, ch):
    """Scan the grid and return (row, col) of the first cell equal to ch."""
    for r, row in enumerate(grid):
        for c, cell in enumerate(row):
            if cell == ch:
                return (r, c)
    return None


def step_cost(grid, cell):
    """Cost of ENTERING a cell: '~' = 3, everything else walkable = 1."""
    return 3 if grid[cell[0]][cell[1]] == "~" else 1


def neighbors(grid, cell):
    """Yield walkable neighbours (Up, Down, Left, Right) that are not walls."""
    r, c = cell
    for dr, dc in MOVES:
        nr, nc = r + dr, c + dc
        if 0 <= nr < len(grid) and 0 <= nc < len(grid[nr]) and grid[nr][nc] != "#":
            yield (nr, nc)


def manhattan(a, b):
    """Heuristic: Manhattan distance. Admissible & consistent because the
    cheapest possible move costs 1 and the robot moves 4-directionally."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def rebuild_path(parent, goal):
    """Walk back from the goal to the start using the parent dictionary."""
    path, node = [], goal
    while node is not None:
        path.append(node)
        node = parent[node]
    return path[::-1]


def path_cost(grid, path):
    """Sum of entering costs (start cell is not counted)."""
    return sum(step_cost(grid, p) for p in path[1:])


def result(path, grid, expanded, max_frontier, elapsed):
    """Pack the search results into one dictionary."""
    return {"path": path,
            "cost": path_cost(grid, path) if path else None,
            "expanded": expanded,
            "max_frontier": max_frontier,
            "time_ms": elapsed * 1000}


# ---------------------- A*  (informed search) ----------------------
def astar(grid, start, goal):
    """A*: expands the node with the lowest f = g + h (Manhattan)."""
    t0 = time.perf_counter()
    frontier = [(manhattan(start, goal), 0, start)]   # (f, g, cell)
    best_g = {start: 0}
    parent = {start: None}
    closed = set()
    expanded, max_f = 0, 1
    while frontier:
        f, g, node = heapq.heappop(frontier)
        if node in closed:
            continue
        closed.add(node)
        expanded += 1
        if node == goal:
            return result(rebuild_path(parent, goal), grid, expanded, max_f,
                          time.perf_counter() - t0)
        for nb in neighbors(grid, node):
            ng = g + step_cost(grid, nb)
            if nb not in best_g or ng < best_g[nb]:
                best_g[nb] = ng
                parent[nb] = node
                heapq.heappush(frontier, (ng + manhattan(nb, goal), ng, nb))
        max_f = max(max_f, len(frontier))
    return result(None, grid, expanded, max_f, time.perf_counter() - t0)
