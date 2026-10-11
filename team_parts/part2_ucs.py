"""
PART 2 (Member 2): Uniform-Cost Search (UCS).
"""
import heapq
import time

from part1 import (
    neighbors, step_cost, rebuild_path, result, find_cell,
    MAP_A, MAP_B, MAP_C
)


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

# Run the evaluation for UCS (only when this file is run directly)
if __name__ == "__main__":
    run_ucs_on_all_maps()
