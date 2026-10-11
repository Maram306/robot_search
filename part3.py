"""
PART 3 (Member 3): Run both algorithms on all maps, display paths,
and print the final results table + comparison.

Run this AFTER Part 1 and Part 2.
"""

from part1 import (
    astar, find_cell, MAP_A, MAP_B, MAP_C
)
from part2_ucs import ucs


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