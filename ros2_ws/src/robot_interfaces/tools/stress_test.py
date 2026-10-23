#!/usr/bin/env python3
"""
Exhaustive Stress Test - Test algorithm on ALL 138,600 configurations
Find worst cases by: cost, path length, turns, R1 to clear
"""

import sys
import time
from itertools import combinations
from collections import deque
import numpy as np

# ============== ALGORITHM (copied from demo) ==============

ENTRY = (-1, 1)
ENTRY_BLOCK = (0, 1)
EXITS = [(3, 0), (3, 2)]

def get_height_map(team_side='RED'):
    if team_side == 'RED':
        return {
            (0,0): 400, (0,1): 200, (0,2): 400,
            (1,0): 200, (1,1): 400, (1,2): 600,
            (2,0): 400, (2,1): 600, (2,2): 400,
            (3,0): 200, (3,1): 400, (3,2): 200
        }
    else:
        return {
            (0,0): 400, (0,1): 200, (0,2): 400,
            (1,0): 600, (1,1): 400, (1,2): 200,
            (2,0): 400, (2,1): 600, (2,2): 400,
            (3,0): 200, (3,1): 400, (3,2): 200
        }

def is_inside_grid(pos):
    return 0 <= pos[0] < 4 and 0 <= pos[1] < 3

def can_r1_reach(pos):
    """R1 can reach all blocks except 5 and 8 (center of middle rows)"""
    r, c = pos
    # Row 0 (entry) and Row 3 (exit) - all columns reachable
    if r == 0 or r == 3:
        return True
    # Rows 1 and 2 - only edge columns (0 and 2) reachable
    return c == 0 or c == 2

def is_passable(grid, pos, collected_kfs=None):
    if not is_inside_grid(pos):
        return False
    val = grid[pos[0]][pos[1]]
    if val == 0:
        return True
    if val == 1:
        return can_r1_reach(pos)
    if val == 2 and collected_kfs and pos in collected_kfs:
        return True
    return False

def get_adjacent(pos):
    r, c = pos
    adj = []
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if is_inside_grid((nr, nc)):
            adj.append((nr, nc))
    return adj

def get_normal_pickup_positions(grid, kfs_pos, collected_kfs=None):
    pickups = []
    for adj in get_adjacent(kfs_pos):
        if is_passable(grid, adj, collected_kfs):
            pickups.append(adj)
    return pickups

def bfs_path(grid, start, end, collected_kfs=None):
    if start == ENTRY:
        start = ENTRY_BLOCK
    if start == end:
        return [start]
    
    queue = deque([(start, [start])])
    visited = {start}
    
    while queue:
        pos, path = queue.popleft()
        for adj in get_adjacent(pos):
            if adj == end:
                return path + [adj]
            if adj not in visited and is_passable(grid, adj, collected_kfs):
                visited.add(adj)
                queue.append((adj, path + [adj]))
    return None

def calculate_path_cost(path, heights):
    if not path or len(path) < 2:
        return 0
    cost = 0
    for i in range(len(path) - 1):
        if not is_inside_grid(path[i]) or not is_inside_grid(path[i+1]):
            continue
        h1 = heights.get(path[i], 200)
        h2 = heights.get(path[i+1], 200)
        cost += 1  # Base movement
        if h2 > h1:
            cost += (h2 - h1) / 200 * 0.5  # Climbing penalty
    return cost

def count_turns(path):
    if len(path) < 3:
        return 0
    turns = 0
    for i in range(len(path) - 2):
        if not is_inside_grid(path[i]) or not is_inside_grid(path[i+1]) or not is_inside_grid(path[i+2]):
            continue
        dir1 = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
        dir2 = (path[i+2][0] - path[i+1][0], path[i+2][1] - path[i+1][1])
        if dir1 != dir2:
            turns += 1
    return turns

def try_build_route(grid, heights, kfs1, pickup1, enter_at, kfs2, pickup2, exit_pos, collected):
    path_segments = []
    
    if pickup1 == ENTRY:
        seg1 = [enter_at]
    else:
        seg1 = bfs_path(grid, ENTRY, pickup1, collected_kfs=set())
        if not seg1:
            return None
    path_segments.append(seg1)
    
    collected_1 = {kfs1}
    start_pos = seg1[-1]
    seg2 = bfs_path(grid, start_pos, pickup2, collected_kfs=collected_1)
    if not seg2:
        return None
    path_segments.append(seg2[1:] if len(seg2) > 1 else [])
    
    collected_2 = {kfs1, kfs2}
    seg3 = bfs_path(grid, pickup2, exit_pos, collected_kfs=collected_2)
    if not seg3:
        return None
    path_segments.append(seg3[1:] if len(seg3) > 1 else [])
    
    full_path = []
    for seg in path_segments:
        full_path.extend(seg)
    
    cost = calculate_path_cost(full_path, heights)
    turns = count_turns(full_path)
    
    r1_to_clear = list(set([
        pos[0]*3+pos[1]+1 for pos in full_path 
        if is_inside_grid(pos) and grid[pos[0]][pos[1]] == 1
    ]))
    
    return {
        'path': full_path,
        'kfs_order': [kfs1, kfs2],
        'pickup_positions': [pickup1 if pickup1 != ENTRY else enter_at, pickup2],
        'first_from_entrance': pickup1 == ENTRY,
        'cost': cost,
        'turns': turns,
        'path_length': len(full_path),
        'r1_to_clear': sorted(r1_to_clear)
    }

def try_first_kfs_route(grid, heights, first_kfs, first_pickup, enter_at, r2_kfs, best_cost):
    collected_1 = {first_kfs}
    remaining = [k for k in r2_kfs if k != first_kfs]
    
    best_route = None
    for second_kfs in remaining:
        pickups2 = get_normal_pickup_positions(grid, second_kfs, collected_1)
        if not pickups2:
            continue
        
        for pickup2 in pickups2:
            for exit_pos in EXITS:
                result = try_build_route(
                    grid, heights, first_kfs, first_pickup, enter_at,
                    second_kfs, pickup2, exit_pos, collected_1
                )
                if result and result['cost'] < best_cost:
                    best_cost = result['cost']
                    best_route = result
    
    return best_route

def find_optimal_route(grid, heights):
    r2_kfs = [(r, c) for r in range(4) for c in range(3) if grid[r][c] == 2]
    
    if len(r2_kfs) < 2:
        return None, "Need at least 2 R2 KFS!"
    
    entry_row_kfs = [k for k in r2_kfs if k[0] == 0]
    
    if entry_row_kfs:
        first_candidates = entry_row_kfs
    else:
        first_candidates = r2_kfs
    
    best_cost = float('inf')
    best_route = None
    
    for first_kfs in first_candidates:
        if first_kfs[0] == 0:
            first_pickup = ENTRY
            enter_at = ENTRY_BLOCK
            result = try_first_kfs_route(grid, heights, first_kfs, first_pickup,
                                         enter_at, r2_kfs, best_cost)
            if result and result['cost'] < best_cost:
                best_cost = result['cost']
                best_route = result
        else:
            pickups = get_normal_pickup_positions(grid, first_kfs)
            for first_pickup in pickups:
                enter_at = first_pickup
                result = try_first_kfs_route(grid, heights, first_kfs, first_pickup,
                                             enter_at, r2_kfs, best_cost)
                if result and result['cost'] < best_cost:
                    best_cost = result['cost']
                    best_route = result
    
    if best_route is None:
        # Check if illegal config (R1 on 5 or 8)
        is_illegal = grid[1][1] == 1 or grid[2][1] == 1
        
        if is_illegal:
            # Try drop-to-clear strategy
            drop_route = try_drop_to_clear(grid, heights, r2_kfs, first_candidates)
            if drop_route:
                drop_route['fallback'] = True
                drop_route['uses_drop'] = True
                return drop_route, None
        
        # FALLBACK: Try 1-KFS route
        fallback_route = try_one_kfs_fallback(grid, heights, r2_kfs, first_candidates)
        if fallback_route:
            fallback_route['fallback'] = True
            fallback_route['uses_drop'] = False
            return fallback_route, None
        return None, "No valid route found!"
    
    best_route['fallback'] = False
    best_route['uses_drop'] = False
    return best_route, None

def try_drop_to_clear(grid, heights, r2_kfs, first_candidates):
    """Drop-to-clear: pick blocker, drop it, pick targets, exit."""
    import copy
    best_cost = float('inf')
    best_route = None
    
    for first_kfs in first_candidates:
        if first_kfs[0] != 0:
            continue
        
        remaining_r2 = [k for k in r2_kfs if k != first_kfs]
        collected_with_first = {first_kfs}
        
        for second_target in remaining_r2:
            for blocker in remaining_r2:
                if blocker == second_target:
                    continue
                
                blocker_pickups = get_normal_pickup_positions(grid, blocker, collected_with_first)
                for blocker_pickup in blocker_pickups:
                    path1 = bfs_path(grid, ENTRY_BLOCK, blocker_pickup, collected_kfs=collected_with_first)
                    if not path1:
                        continue
                    
                    for drop_pos in get_adjacent(blocker_pickup):
                        if not is_inside_grid(drop_pos) or grid[drop_pos[0]][drop_pos[1]] != 0:
                            continue
                        
                        temp_grid = copy.deepcopy(grid)
                        temp_grid[blocker[0]][blocker[1]] = 0
                        temp_grid[drop_pos[0]][drop_pos[1]] = 2
                        
                        target_pickups = get_normal_pickup_positions(temp_grid, second_target, collected_with_first)
                        for target_pickup in target_pickups:
                            path2 = bfs_path(temp_grid, blocker_pickup, target_pickup, collected_kfs=collected_with_first)
                            if not path2:
                                continue
                            
                            final_collected = collected_with_first | {second_target}
                            for exit_pos in EXITS:
                                path3 = bfs_path(temp_grid, target_pickup, exit_pos, collected_kfs=final_collected)
                                if not path3:
                                    continue
                                
                                full_path = path1 + path2[1:] + path3[1:]
                                cost = calculate_path_cost(full_path, heights) + 2
                                
                                if cost < best_cost:
                                    best_cost = cost
                                    best_route = {
                                        'path': full_path,
                                        'kfs_order': [first_kfs, second_target],
                                        'cost': cost,
                                        'path_length': len(full_path),
                                        'turns': count_turns(full_path),
                                        'r1_to_clear': [],
                                        'kfs_count': 2
                                    }
    return best_route

def try_one_kfs_fallback(grid, heights, r2_kfs, first_candidates):
    """Fallback: pick 1 KFS and exit if 2-KFS impossible."""
    best_cost = float('inf')
    best_route = None
    
    for kfs in first_candidates:
        if kfs[0] == 0:  # Entry row
            collected = {kfs}
            for exit_pos in EXITS:
                path = bfs_path(grid, ENTRY_BLOCK, exit_pos, collected_kfs=collected)
                if path:
                    cost = calculate_path_cost(path, heights)
                    if cost < best_cost:
                        best_cost = cost
                        r1_to_clear = [pos[0]*3+pos[1]+1 for pos in path 
                                       if is_inside_grid(pos) and grid[pos[0]][pos[1]] == 1]
                        best_route = {
                            'path': path,
                            'kfs_order': [kfs],
                            'cost': cost,
                            'path_length': len(path),
                            'turns': count_turns(path),
                            'r1_to_clear': list(set(r1_to_clear)),
                            'kfs_count': 1
                        }
    return best_route

# ============== TEST RUNNER ==============

def config_to_grid(r1_blocks, r2_blocks, fake_block):
    """Convert block numbers to grid"""
    grid = [[0]*3 for _ in range(4)]
    for b in r1_blocks:
        r, c = (b-1)//3, (b-1)%3
        grid[r][c] = 1
    for b in r2_blocks:
        r, c = (b-1)//3, (b-1)%3
        grid[r][c] = 2
    r, c = (fake_block-1)//3, (fake_block-1)%3
    grid[r][c] = -1
    return grid

def run_exhaustive_test():
    all_blocks = set(range(1, 13))
    heights = get_height_map('RED')
    
    legal_results = []
    illegal_results = []
    unsolvable = []
    
    total = 0
    for r1_combo in combinations(all_blocks, 3):
        remaining = all_blocks - set(r1_combo)
        for fake in remaining:
            remaining2 = remaining - {fake}
            for r2_combo in combinations(remaining2, 4):
                total += 1
    
    print(f"Testing {total} configurations...")
    print()
    
    tested = 0
    start_time = time.time()
    
    for r1_combo in combinations(all_blocks, 3):
        remaining = all_blocks - set(r1_combo)
        for fake in remaining:
            remaining2 = remaining - {fake}
            for r2_combo in combinations(remaining2, 4):
                tested += 1
                
                if tested % 10000 == 0:
                    elapsed = time.time() - start_time
                    rate = tested / elapsed
                    remaining_time = (total - tested) / rate
                    print(f"  {tested}/{total} ({100*tested/total:.1f}%) - ETA: {remaining_time:.0f}s")
                
                grid = config_to_grid(r1_combo, r2_combo, fake)
                is_illegal = 5 in r1_combo or 8 in r1_combo  # R1 on unreachable blocks
                route, error = find_optimal_route(grid, heights)
                
                config = {
                    'r1': sorted(r1_combo),
                    'r2': sorted(r2_combo),
                    'fake': fake
                }
                
                if route:
                    result = {
                        'config': config,
                        'cost': route['cost'],
                        'path_length': route['path_length'],
                        'turns': route['turns'],
                        'r1_to_clear': route.get('r1_to_clear', []),
                        'kfs_order': [k[0]*3+k[1]+1 for k in route['kfs_order']],
                        'path': route['path'],
                        'fallback': route.get('fallback', False),
                        'uses_drop': route.get('uses_drop', False)
                    }
                    if is_illegal:
                        illegal_results.append(result)
                    else:
                        legal_results.append(result)
                else:
                    unsolvable.append({
                        'config': config,
                        'error': error,
                        'is_illegal': is_illegal
                    })
    
    elapsed = time.time() - start_time
    print(f"\nCompleted in {elapsed:.1f} seconds ({total/elapsed:.0f} configs/sec)")
    print()
    
    # Analysis
    print("=" * 60)
    print("RESULTS ANALYSIS")
    print("=" * 60)
    print()
    
    print(f"Total configs tested: {total}")
    print(f"Legal configs: {len(legal_results)} solvable")
    print(f"Illegal configs: {len(illegal_results)} solvable, {len([u for u in unsolvable if u.get('is_illegal')])} unsolvable")
    print(f"Total solvable: {len(legal_results) + len(illegal_results)}")
    print(f"Total unsolvable: {len(unsolvable)}")
    print()
    
    all_results = legal_results + illegal_results
    if not all_results:
        print("No solvable configurations found!")
        return
    
    # Sort legal results
    legal_by_cost = sorted(legal_results, key=lambda x: x['cost'], reverse=True)
    legal_by_path = sorted(legal_results, key=lambda x: x['path_length'], reverse=True)
    
    # Sort illegal results
    illegal_by_cost = sorted(illegal_results, key=lambda x: x['cost'], reverse=True)
    
    def print_config(r, rank):
        c = r['config']
        drop_tag = " [DROP]" if r.get('uses_drop') else ""
        print(f"  #{rank}: R1={c['r1']} R2={c['r2']} Fake={c['fake']}{drop_tag}")
        print(f"       Cost={r['cost']:.2f} Path={r['path_length']} Turns={r['turns']}")
        print()
    
    print("-" * 40)
    print("TOP 5 WORST LEGAL CONFIGS (by cost):")
    print("-" * 40)
    for i, r in enumerate(legal_by_cost[:5]):
        print_config(r, i+1)
    
    if illegal_results:
        print("-" * 40)
        print("TOP 5 WORST ILLEGAL CONFIGS (by cost):")
        print("-" * 40)
        for i, r in enumerate(illegal_by_cost[:5]):
            print_config(r, i+1)
    
    # Stats
    print()
    print("=" * 60)
    print("STATISTICS")
    print("=" * 60)
    
    if legal_results:
        legal_costs = [r['cost'] for r in legal_results]
        print(f"LEGAL - Cost: min={min(legal_costs):.2f} max={max(legal_costs):.2f}")
    
    if illegal_results:
        illegal_costs = [r['cost'] for r in illegal_results]
        uses_drop = sum(1 for r in illegal_results if r.get('uses_drop'))
        print(f"ILLEGAL - Cost: min={min(illegal_costs):.2f} max={max(illegal_costs):.2f}")
        print(f"ILLEGAL - Using drop-to-clear: {uses_drop}")
    
    if unsolvable:
        print()
        print("-" * 40)
        print(f"UNSOLVABLE CONFIGS (sample first 5):")
        print("-" * 40)
        for i, u in enumerate(unsolvable[:5]):
            c = u['config']
            tag = " [ILLEGAL]" if u.get('is_illegal') else ""
            print(f"  #{i+1}: R1={c['r1']} R2={c['r2']} Fake={c['fake']}{tag}")
            print(f"       Error: {u['error']}")
            print()
    
    # ============== SAVE RESULTS TO FILES ==============
    import os
    import json
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    def format_config(r, include_path=False):
        c = r['config']
        drop_tag = " [DROP]" if r.get('uses_drop') else ""
        lines = [
            f"R1: {c['r1']}  R2: {c['r2']}  Fake: {c['fake']}{drop_tag}",
            f"Cost: {r['cost']:.2f}  Path: {r['path_length']}  Turns: {r['turns']}",
            f"KFS Order: {r['kfs_order']}"
        ]
        if include_path:
            path_blocks = [p[0]*3+p[1]+1 for p in r['path'] if is_inside_grid(p)]
            lines.append(f"Path: {path_blocks}")
        return "\n".join(lines)
    
    # Save top 10 LEGAL by cost
    legal_file = os.path.join(script_dir, "stress_test_top10_legal.txt")
    with open(legal_file, 'w') as f:
        f.write("TOP 10 WORST LEGAL CONFIGS\n")
        f.write("=" * 60 + "\n\n")
        for i, r in enumerate(legal_by_cost[:10]):
            f.write(f"#{i+1}\n{format_config(r, True)}\n\n")
    print(f"Saved: {legal_file}")
    
    # Save top 10 ILLEGAL by cost
    if illegal_results:
        illegal_file = os.path.join(script_dir, "stress_test_top10_illegal.txt")
        with open(illegal_file, 'w') as f:
            f.write("TOP 10 WORST ILLEGAL CONFIGS\n")
            f.write("=" * 60 + "\n\n")
            for i, r in enumerate(illegal_by_cost[:10]):
                f.write(f"#{i+1}\n{format_config(r, True)}\n\n")
        print(f"Saved: {illegal_file}")
    
    # Save unsolvable
    if unsolvable:
        unsolvable_file = os.path.join(script_dir, "stress_test_unsolvable.txt")
        with open(unsolvable_file, 'w') as f:
            f.write(f"UNSOLVABLE CONFIGURATIONS ({len(unsolvable)} total)\n")
            f.write("=" * 60 + "\n\n")
            for i, u in enumerate(unsolvable):
                c = u['config']
                tag = " [ILLEGAL]" if u.get('is_illegal') else ""
                f.write(f"#{i+1}: R1={c['r1']} R2={c['r2']} Fake={c['fake']}{tag}\n")
                f.write(f"     Error: {u['error']}\n\n")
        print(f"Saved: {unsolvable_file}")
    
    # Save summary JSON
    summary_file = os.path.join(script_dir, "stress_test_summary.json")
    summary = {
        'total_tested': total,
        'legal_solvable': len(legal_results),
        'illegal_solvable': len(illegal_results),
        'unsolvable': len(unsolvable),
        'illegal_using_drop': sum(1 for r in illegal_results if r.get('uses_drop')),
        'worst_legal': legal_by_cost[0]['config'] if legal_results else None,
        'worst_illegal': illegal_by_cost[0]['config'] if illegal_results else None
    }
    with open(summary_file, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"Saved: {summary_file}")
    
    print()
    print("=" * 60)
    print("ALL FILES SAVED TO tools/ DIRECTORY")
    print("=" * 60)

if __name__ == "__main__":
    run_exhaustive_test()

