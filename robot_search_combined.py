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


"""
PART 2 (Member 2): Uniform-Cost Search (UCS).
"""
# ---------------------- UCS (Uniform Cost Search) ----------------------
def ucs(grid, start, goal):
    """UCS: Uniform Cost Search, expands the node with the lowest path cost g(n)."""
    t0 = time.perf_counter()
    frontier = [(0, start)]   # (g, cell)
    best_g = {start: 0}
    parent = {start: None}
    closed = set()
    expanded, max_f = 0, 1
    
    while frontier:
        g, node = heapq.heappop(frontier)
        
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
                heapq.heappush(frontier, (ng, nb))
                
        max_f = max(max_f, len(frontier))
        
    return result(None, grid, expanded, max_f, time.perf_counter() - t0)


def print_map_with_path(grid, path):
    """Displays the map with the path marked by '*' (excluding S and G if preferred, or covering them)."""
    if not path:
        print("No path to display.")
        return
    
    # Convert grid to a list of lists of characters for modification
    grid_copy = [list(row) for row in grid]
    
    for r, c in path:
        # Keep S and G symbols visible, or mark path cells with '*'
        if grid_copy[r][c] not in ('S', 'G'):
            grid_copy[r][c] = '*'
            
    for row in grid_copy:
        print("".join(row))
    print()


def run_ucs_on_all_maps():
    """Runs UCS on MAP_A, MAP_B, and MAP_C as required by the assignment."""
    maps = {"MAP_A": MAP_A, "MAP_B": MAP_B, "MAP_C": MAP_C}
    
    print("=" * 60)
    print(f"{'ALGORITHM':<8} | {'MAP':<6} | {'COST':<5} | {'EXPANDED':<8} | {'MAX FRONTIER':<12} | {'TIME (ms)':<8}")
    print("-" * 60)
    
    for map_name, grid in maps.items():
        start = find_cell(grid, "S")
        goal = find_cell(grid, "G")
        
        res = ucs(grid, start, goal)
        
        if res["path"]:
            print(f"{'UCS':<8} | {map_name:<6} | {res['cost']:<5} | {res['expanded']:<8} | {res['max_frontier']:<12} | {res['time_ms']:.3f}")
            print(f"\nPath on {map_name}:")
            print_map_with_path(grid, res["path"])
        else:
            print(f"{'UCS':<8} | {map_name:<6} | {'None':<5} | {res['expanded']:<8} | {res['max_frontier']:<12} | {res['time_ms']:.3f}")
            print(f"\nMAP_C Result: No path found.\n")


"""
PART 3 (Member 3): Run both algorithms on all maps, display paths,
and print the final results table + comparison.

Run this AFTER Part 1 and Part 2 (same notebook).
"""



# ============================================================
# 1. Display Helper
# ============================================================
def print_map_with_path(grid, path, title=""):
    """Print the grid with the path marked by '*' (S and G kept visible)."""
    if not path:
        print(f"  {title}: No path to display.")
        return

    grid_copy = [list(row) for row in grid]
    for r, c in path:
        if grid_copy[r][c] not in ('S', 'G'):
            grid_copy[r][c] = '*'

    print(f"  {title}:")
    for row in grid_copy:
        print("    " + "".join(row))
    print()


# ============================================================
# 2. Run a single algorithm on a single map
# ============================================================
def run_one(name, grid, algo_name, algo_func):
    """Run one algorithm on one map and return a results row."""
    start = find_cell(grid, "S")
    goal = find_cell(grid, "G")
    res = algo_func(grid, start, goal)

    path = res["path"]
    length = (len(path) - 1) if path else None  # number of moves

    return {
        "map": name,
        "algo": algo_name,
        "length": length,
        "cost": res["cost"],
        "expanded": res["expanded"],
        "max_frontier": res["max_frontier"],
        "time_ms": res["time_ms"],
        "path": path,
        "grid": grid,
    }


# ============================================================
# 3. Main
# ============================================================
def main():
    maps = [("MAP_A", MAP_A), ("MAP_B", MAP_B), ("MAP_C", MAP_C)]
    algorithms = [("UCS", ucs), ("A*", astar)]

    all_results = []

    for map_name, grid in maps:
        print("=" * 70)
        print(f"MAP: {map_name}")
        print("=" * 70)

        for algo_name, algo_func in algorithms:
            row = run_one(map_name, grid, algo_name, algo_func)
            all_results.append(row)

            print(f"\n--- {algo_name} on {map_name} ---")
            if row["path"]:
                print(f"  Path length : {row['length']} moves")
                print(f"  Path cost   : {row['cost']}")
                print(f"  Path        : {row['path']}")
            else:
                print(f"  No path found.")
            print(f"  Nodes expanded : {row['expanded']}")
            print(f"  Max frontier   : {row['max_frontier']}")
            print(f"  Run time       : {row['time_ms']:.3f} ms")

            print_map_with_path(grid, row["path"],
                                title=f"{algo_name} path on {map_name}")

    # ============================================================
    # 4. Final Results Table
    # ============================================================
    print("\n" + "=" * 90)
    print("FINAL RESULTS TABLE")
    print("=" * 90)
    header = (f"{'Map':<8} | {'Algo':<5} | {'Length':<7} | {'Cost':<6} | "
              f"{'Expanded':<9} | {'Max Frontier':<13} | {'Time (ms)':<9}")
    print(header)
    print("-" * 90)

    for r in all_results:
        length = r["length"] if r["length"] is not None else "None"
        cost = r["cost"] if r["cost"] is not None else "None"
        print(f"{r['map']:<8} | {r['algo']:<5} | {length:<7} | {cost:<6} | "
              f"{r['expanded']:<9} | {r['max_frontier']:<13} | {r['time_ms']:<9.3f}")

    # ============================================================
    # 5. Comparison
    # ============================================================
    print("\n" + "=" * 90)
    print("COMPARISON: UCS vs A*")
    print("=" * 90)
    for map_name in ["MAP_A", "MAP_B", "MAP_C"]:
        ucs_row = next((r for r in all_results
                        if r["map"] == map_name and r["algo"] == "UCS"), None)
        astar_row = next((r for r in all_results
                          if r["map"] == map_name and r["algo"] == "A*"), None)
        if ucs_row and astar_row:
            print(f"\n{map_name}:")
            print(f"  UCS  -> Cost: {ucs_row['cost']}, Expanded: {ucs_row['expanded']}")
            print(f"  A*   -> Cost: {astar_row['cost']}, Expanded: {astar_row['expanded']}")
            if ucs_row['expanded'] and astar_row['expanded']:
                saved = ucs_row['expanded'] - astar_row['expanded']
                if saved > 0:
                    print(f"  A* expanded {saved} fewer nodes.")
                else:
                    print("  Both expanded the same number of nodes.")

    print("\nConclusion:")
    print("  - Both UCS and A* find the cheapest path.")
    print("  - A* expands fewer nodes than UCS on MAP_A and MAP_B thanks to the")
    print("    Manhattan heuristic. On MAP_C (no path) both expand the same nodes.")
    print("  - For a warehouse robot, A* is preferred: same optimal cost with")
    print("    fewer expanded nodes (at the price of a slightly larger frontier).")


if __name__ == "__main__":
    main()
