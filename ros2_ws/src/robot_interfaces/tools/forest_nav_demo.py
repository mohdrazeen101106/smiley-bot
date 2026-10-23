#!/usr/bin/env python3
"""
Forest Navigation Algorithm - FINAL VERSION
All rules correctly enforced:
  1. First KFS must be from blocks 1,2,3 if any contain R2 KFS
  2. First pickup is from ENTRANCE ZONE (special case)
  3. Second+ pickups must be from INSIDE forest (adjacent block)
  4. Cannot pick exit KFS (10,11,12) from exit zone - must be inside
  5. Avoids Fake (-1) and R2 KFS (2) blocks
  6. R1 KFS (1) are passable (R1 will clear)

Controls:
  Click: cycle values | R: team | C: compute | S: animate | Q: quit
"""

import cv2
import numpy as np
import math
from itertools import combinations
from collections import deque

# ============== CONSTANTS ==============

ENTRY = (-1, 1)  # ENTRANCE ZONE - OUTSIDE the grid!
ENTRY_BLOCK = (0, 1)  # First block R2 steps onto (Block 2)
EXITS = [(3, 0), (3, 2)]  # Block 10 and Block 12

# ============== ALGORITHM CODE ==============

def get_height_map(team_side):
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
    """Check if position is inside the 4x3 forest grid"""
    return 0 <= pos[0] < 4 and 0 <= pos[1] < 3

def can_r1_reach(pos):
    """
    Check if R1 can reach this block from the pathway.
    R1 travels on pathways around the forest perimeter.
    
    Reachable blocks (adjacent to pathway):
    - Row 0 (entry): blocks 1, 2, 3 - all adjacent to entry pathway
    - Row 1: blocks 4, 6 - edges adjacent to side pathways
    - Row 2: blocks 7, 9 - edges adjacent to side pathways
    - Row 3 (exit): blocks 10, 11, 12 - all adjacent to exit pathway
    
    UNREACHABLE blocks (center, not adjacent to any pathway):
    - Block 5 (row 1, col 1) and Block 8 (row 2, col 1)
    """
    r, c = pos
    # Row 0 (entry) and Row 3 (exit) - all columns reachable
    if r == 0 or r == 3:
        return True
    # Rows 1 and 2 - only edge columns (0 and 2) reachable
    return c == 0 or c == 2

def is_passable(grid, pos, collected_kfs=None):
    """
    Passable cells:
    - Empty (0)
    - R1 KFS (1) on EDGE columns (R1 can clear)
    - Collected R2 KFS
    
    Blocked:
    - Fake (-1)
    - Uncollected R2 KFS (2)
    - R1 KFS (1) on MIDDLE column (R1 cannot clear!)
    """
    if not is_inside_grid(pos):
        return False
    
    val = grid[pos[0]][pos[1]]
    
    if val == 0:  # Empty
        return True
    
    if val == 1:  # R1 KFS
        # Only passable if R1 can reach it (edge columns)
        return can_r1_reach(pos)
    
    if val == 2 and collected_kfs and pos in collected_kfs:
        return True  # Already collected
    
    return False

def get_adjacent(pos):
    """Get 4-connected adjacent positions inside grid"""
    r, c = pos
    adj = []
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if is_inside_grid((nr, nc)):
            adj.append((nr, nc))
    return adj

def get_first_pickup_info(grid, kfs_pos):
    """
    FIRST KFS pickup - special rules:
    - If KFS in row 0 (blocks 1,2,3): pickup from ENTRANCE ZONE
    - Otherwise: pickup from adjacent block inside grid
    
    Returns: (pickup_position, enters_at_block)
    """
    if kfs_pos[0] == 0:  # Row 0 = blocks 1,2,3
        # First pickup from entrance zone!
        # R2 picks while in entrance zone, then enters at block 2
        return ENTRY, ENTRY_BLOCK
    else:
        # First KFS not in row 0 - must enter first, then pick
        # Enter at block 2, then navigate to adjacent position
        pickups = []
        for adj in get_adjacent(kfs_pos):
            if is_passable(grid, adj):
                pickups.append((adj, adj))  # (pickup_pos, enters_at)
        return pickups if pickups else None

def get_normal_pickup_positions(grid, kfs_pos, collected_kfs=None):
    """
    Normal pickup (not first) - must be on adjacent block INSIDE grid.
    Cannot use entrance or exit zones.
    """
    pickups = []
    for adj in get_adjacent(kfs_pos):
        if is_passable(grid, adj, collected_kfs):
            pickups.append(adj)
    return pickups

def bfs_path(grid, start, end, collected_kfs=None):
    """Find shortest path avoiding blocked cells."""
    if start == end:
        return [start]
    
    # Special case: start from entrance zone
    if start == ENTRY:
        # Entrance connects to block 2 (0,1)
        # Start BFS from the entry block
        start = ENTRY_BLOCK
        if not is_passable(grid, start, collected_kfs):
            return None
        initial_path = [start]
    else:
        if not is_passable(grid, start, collected_kfs):
            return None
        initial_path = [start]
    
    if not is_passable(grid, end, collected_kfs):
        return None
    
    if start == end:
        return initial_path
    
    queue = deque([(start, initial_path)])
    visited = {start}
    
    while queue:
        (r, c), path = queue.popleft()
        
        for next_pos in get_adjacent((r, c)):
            if next_pos in visited:
                continue
            if not is_passable(grid, next_pos, collected_kfs):
                continue
            
            new_path = path + [next_pos]
            
            if next_pos == end:
                return new_path
            
            visited.add(next_pos)
            queue.append((next_pos, new_path))
    
    return None

def calculate_path_cost(path, heights):
    """Cost = distance + height penalty"""
    if not path or len(path) < 1:
        return float('inf')
    
    # Filter out entrance zone position if present
    grid_path = [p for p in path if is_inside_grid(p)]
    
    if len(grid_path) < 2:
        return 1.0  # Minimal cost for single block
    
    distance = len(grid_path) - 1
    height_penalty = 0
    
    for i in range(len(grid_path) - 1):
        delta = abs(heights[grid_path[i]] - heights[grid_path[i+1]])
        if delta >= 400:
            height_penalty += 1.5
        elif delta >= 200:
            height_penalty += 0.5
    
    return distance + height_penalty

def find_optimal_route(grid, heights):
    """
    Find optimal 2-KFS route following ALL rules:
    1. First KFS from entry row (1,2,3) if any contain R2 KFS
    2. First pickup from entrance zone (for row 0 KFS)
    3. Second pickup from inside grid only
    4. Proper pathfinding avoiding blocked cells
    """
    # Find all R2 KFS
    r2_kfs = [(r, c) for r in range(4) for c in range(3) if grid[r][c] == 2]
    
    if len(r2_kfs) < 2:
        return None, "Need at least 2 R2 KFS!"
    
    # Check if any R2 KFS in entry row (row 0 = blocks 1,2,3)
    entry_row_kfs = [k for k in r2_kfs if k[0] == 0]
    
    if entry_row_kfs:
        # Rule 4.4.15: FIRST KFS must be from entry row
        first_candidates = entry_row_kfs
    else:
        # No KFS in entry row - first can be any
        first_candidates = r2_kfs
    
    best_cost = float('inf')
    best_route = None
    
    for first_kfs in first_candidates:
        # Get first pickup position
        first_pickup_info = get_first_pickup_info(grid, first_kfs)
        
        if first_kfs[0] == 0:
            # Entry row - pickup from entrance zone
            first_pickup = ENTRY
            enter_at = ENTRY_BLOCK
        else:
            # Not entry row - need to navigate to adjacent position
            pickups = get_normal_pickup_positions(grid, first_kfs)
            if not pickups:
                continue
            # Will try each pickup position below
            for first_pickup in pickups:
                enter_at = first_pickup
                result = try_first_kfs_route(grid, heights, first_kfs, first_pickup, 
                                             enter_at, r2_kfs, best_cost)
                if result and result['cost'] < best_cost:
                    best_cost = result['cost']
                    best_route = result
            continue
        
        # For entry row KFS
        result = try_first_kfs_route(grid, heights, first_kfs, first_pickup,
                                     enter_at, r2_kfs, best_cost)
        if result and result['cost'] < best_cost:
            best_cost = result['cost']
            best_route = result
    
    if best_route is None:
        # Check if this is an illegal config (R1 on blocks 5 or 8)
        is_illegal = has_r1_on_unreachable(grid)
        
        if is_illegal:
            # Try drop-to-clear strategy for illegal configs
            drop_route = try_drop_to_clear(grid, heights, r2_kfs, first_candidates)
            if drop_route:
                drop_route['fallback'] = True
                drop_route['fallback_reason'] = "Illegal config - using drop-to-clear"
                drop_route['is_legal'] = False
                return drop_route, None
        
        # Standard 1-KFS fallback
        fallback_route = try_one_kfs_fallback(grid, heights, r2_kfs, first_candidates)
        if fallback_route:
            fallback_route['fallback'] = True
            fallback_route['fallback_reason'] = "2-KFS path blocked, using 1-KFS fallback"
            fallback_route['is_legal'] = not is_illegal
            return fallback_route, None
        return None, "No valid route found!"
    
    best_route['fallback'] = False
    best_route['is_legal'] = not has_r1_on_unreachable(grid)
    return best_route, None

def has_r1_on_unreachable(grid):
    """Check if R1 KFS is on blocks 5 or 8 (unreachable by R1)."""
    # Block 5 = (1, 1), Block 8 = (2, 1)
    return grid[1][1] == 1 or grid[2][1] == 1

def try_drop_to_clear(grid, heights, r2_kfs, first_candidates):
    """
    DROP-TO-CLEAR strategy for illegal configs.
    
    Strategy:
    1. Pick first KFS from entry
    2. Navigate towards exit, picking up blocking R2 KFS
    3. Drop blocking KFS on empty block
    4. Continue to pick second target KFS
    5. Exit with 2 KFS
    """
    import copy
    
    best_cost = float('inf')
    best_route = None
    
    for first_kfs in first_candidates:
        if first_kfs[0] != 0:
            continue  # Only entry row KFS for first
        
        # After picking first KFS, find blockers and drop locations
        collected = {first_kfs}
        
        # Find R2 KFS that block paths
        remaining_r2 = [k for k in r2_kfs if k != first_kfs]
        
        for second_target in remaining_r2:
            # Find blockers between entry and second target
            for blocker in remaining_r2:
                if blocker == second_target:
                    continue
                
                # Try picking up blocker, dropping it, then continuing
                modified_grid = copy.deepcopy(grid)
                
                # Simulate: after collecting first_kfs
                collected_with_first = {first_kfs}
                
                # Path to blocker
                blocker_pickups = get_normal_pickup_positions(grid, blocker, collected_with_first)
                for blocker_pickup in blocker_pickups:
                    path1 = bfs_path(grid, ENTRY_BLOCK, blocker_pickup, collected_kfs=collected_with_first)
                    if not path1:
                        continue
                    
                    # Now holding blocker - find empty drop location
                    collected_with_blocker = collected_with_first | {blocker}
                    
                    # Find adjacent empty blocks to drop
                    for drop_pos in get_adjacent(blocker_pickup):
                        if not is_inside_grid(drop_pos):
                            continue
                        if grid[drop_pos[0]][drop_pos[1]] != 0:  # Not empty
                            continue
                        
                        # Drop blocker here, modify grid
                        temp_grid = copy.deepcopy(grid)
                        temp_grid[blocker[0]][blocker[1]] = 0  # Blocker removed
                        temp_grid[drop_pos[0]][drop_pos[1]] = 2  # Blocker dropped here
                        
                        # Now path to second target
                        collected_after_drop = collected_with_first  # Only first is "kept"
                        target_pickups = get_normal_pickup_positions(temp_grid, second_target, collected_after_drop)
                        
                        for target_pickup in target_pickups:
                            path2 = bfs_path(temp_grid, blocker_pickup, target_pickup, collected_kfs=collected_after_drop)
                            if not path2:
                                continue
                            
                            # After picking second target
                            final_collected = collected_after_drop | {second_target}
                            
                            # Path to exit
                            for exit_pos in EXITS:
                                path3 = bfs_path(temp_grid, target_pickup, exit_pos, collected_kfs=final_collected)
                                if not path3:
                                    continue
                                
                                # Combine paths
                                full_path = path1 + path2[1:] + path3[1:]
                                cost = calculate_path_cost(full_path, heights) + 2  # +2 penalty for drop
                                
                                if cost < best_cost:
                                    best_cost = cost
                                    r1_to_clear = list(set([
                                        pos[0]*3+pos[1]+1 for pos in full_path 
                                        if is_inside_grid(pos) and grid[pos[0]][pos[1]] == 1
                                    ]))
                                    best_route = {
                                        'path': full_path,
                                        'kfs_order': [first_kfs, second_target],
                                        'dropped_kfs': blocker,
                                        'drop_location': drop_pos,
                                        'cost': cost,
                                        'path_length': len(full_path),
                                        'turns': count_turns(full_path),
                                        'r1_to_clear': sorted(r1_to_clear),
                                        'kfs_count': 2,
                                        'uses_drop': True
                                    }
    
    return best_route

def try_one_kfs_fallback(grid, heights, r2_kfs, first_candidates):
    """
    FALLBACK: When 2-KFS route is impossible, try to at least pick 1 KFS and exit.
    This allows the bot to score SOMETHING rather than doing nothing.
    """
    best_cost = float('inf')
    best_route = None
    
    for kfs in first_candidates:
        # Try pickup from entrance if in entry row
        if kfs[0] == 0:
            pickup_pos = ENTRY
            enter_at = ENTRY_BLOCK
            collected = {kfs}
            
            # Try each exit
            for exit_pos in EXITS:
                path_to_exit = bfs_path(grid, enter_at, exit_pos, collected_kfs=collected)
                if path_to_exit:
                    full_path = path_to_exit
                    cost = calculate_path_cost(full_path, heights)
                    
                    if cost < best_cost:
                        best_cost = cost
                        r1_to_clear = list(set([
                            pos[0]*3+pos[1]+1 for pos in full_path 
                            if is_inside_grid(pos) and grid[pos[0]][pos[1]] == 1
                        ]))
                        best_route = {
                            'path': full_path,
                            'kfs_order': [kfs],
                            'kfs_to_pick': [kfs],
                            'pickup_positions': [enter_at],
                            'first_from_entrance': True,
                            'cost': cost,
                            'path_length': len(full_path),
                            'turns': count_turns(full_path) if 'count_turns' in dir() else 0,
                            'r1_to_clear': sorted(r1_to_clear),
                            'kfs_count': 1
                        }
        else:
            # Not in entry row - try adjacent pickups
            pickups = get_normal_pickup_positions(grid, kfs)
            for pickup_pos in pickups:
                path_to_pickup = bfs_path(grid, ENTRY, pickup_pos, collected_kfs=set())
                if not path_to_pickup:
                    continue
                    
                collected = {kfs}
                for exit_pos in EXITS:
                    path_to_exit = bfs_path(grid, pickup_pos, exit_pos, collected_kfs=collected)
                    if path_to_exit:
                        full_path = path_to_pickup + path_to_exit[1:]
                        cost = calculate_path_cost(full_path, heights)
                        
                        if cost < best_cost:
                            best_cost = cost
                            r1_to_clear = list(set([
                                pos[0]*3+pos[1]+1 for pos in full_path 
                                if is_inside_grid(pos) and grid[pos[0]][pos[1]] == 1
                            ]))
                            best_route = {
                                'path': full_path,
                                'kfs_order': [kfs],
                                'kfs_to_pick': [kfs],
                                'pickup_positions': [pickup_pos],
                                'first_from_entrance': False,
                                'cost': cost,
                                'path_length': len(full_path),
                                'turns': 0,
                                'r1_to_clear': sorted(r1_to_clear),
                                'kfs_count': 1
                            }
    
    return best_route


def try_first_kfs_route(grid, heights, first_kfs, first_pickup, enter_at, r2_kfs, best_cost):
    """Try routes with a given first KFS and pickup position."""
    
    best_route = None
    
    # After first pickup, this KFS is collected
    collected_after_1 = {first_kfs}
    
    # Try each second KFS
    for second_kfs in r2_kfs:
        if second_kfs == first_kfs:
            continue
        
        # Second pickup MUST be from INSIDE grid (not entrance/exit zone)
        second_pickups = get_normal_pickup_positions(grid, second_kfs, collected_after_1)
        if not second_pickups:
            continue
        
        for second_pickup in second_pickups:
            # After second pickup, both are collected
            collected_after_2 = {first_kfs, second_kfs}
            
            for exit_pos in EXITS:
                route = try_build_route(
                    grid, heights,
                    first_kfs, first_pickup, enter_at,
                    second_kfs, second_pickup,
                    exit_pos, collected_after_2
                )
                
                if route and route['cost'] < best_cost:
                    best_cost = route['cost']
                    best_route = route
    
    return best_route

def try_build_route(grid, heights, kfs1, pickup1, enter_at, kfs2, pickup2, exit_pos, collected):
    """Build complete route with all segments."""
    
    path_segments = []
    
    # Segment 1: Enter and reach first pickup position
    if pickup1 == ENTRY:
        # First pickup from entrance zone - add enter_at as first grid position
        seg1 = [enter_at]  # Start at entry block after pickup
    else:
        # First pickup inside grid - path from entry to pickup
        seg1 = bfs_path(grid, ENTRY, pickup1, collected_kfs=set())
        if not seg1:
            return None
    
    path_segments.append(seg1)
    
    # After first pickup, kfs1 is collected
    collected_1 = {kfs1}
    
    # Segment 2: First pickup to second pickup
    start_pos = seg1[-1]
    seg2 = bfs_path(grid, start_pos, pickup2, collected_kfs=collected_1)
    if not seg2:
        return None
    
    path_segments.append(seg2[1:] if len(seg2) > 1 else [])
    
    # After second pickup, both collected
    collected_2 = {kfs1, kfs2}
    
    # Segment 3: Second pickup to exit
    seg3 = bfs_path(grid, pickup2, exit_pos, collected_kfs=collected_2)
    if not seg3:
        return None
    
    path_segments.append(seg3[1:] if len(seg3) > 1 else [])
    
    # Combine path
    full_path = []
    for seg in path_segments:
        full_path.extend(seg)
    
    # Calculate cost
    cost = calculate_path_cost(full_path, heights)
    
    # Find R1 KFS on path
    r1_to_clear = list(set([
        pos[0]*3+pos[1]+1 for pos in full_path 
        if is_inside_grid(pos) and grid[pos[0]][pos[1]] == 1
    ]))
    
    return {
        'path': full_path,
        'kfs_order': [kfs1, kfs2],
        'kfs_to_pick': [kfs1, kfs2],
        'pickup_positions': [pickup1 if pickup1 != ENTRY else enter_at, pickup2],
        'first_from_entrance': pickup1 == ENTRY,
        'cost': cost,
        'r1_to_clear': sorted(r1_to_clear)
    }

# ============== VISUALIZATION ==============

class ForestVisualizer:
    def __init__(self):
        self.cell_size = 120
        self.margin = 50
        self.grid = [[0]*3 for _ in range(4)]
        self.team_side = 'RED'
        self.route = None
        self.window_name = "Forest Nav - FINAL"
        
        self.animating = False
        self.robot_pos = None
        self.path_index = 0
        self.animation_progress = 0.0
        self.kfs_collected = []
        self.paused_for_r1 = False
        self.blocking_r1 = None
        self.error_msg = ""  # Validation error message
        
        self.colors = {0: (50,50,50), 2: (0,200,0), 1: (0,100,200), -1: (0,0,200)}
        self.height_colors = {200: (0,150,0), 400: (0,200,100), 600: (100,200,150)}
    
    def count_kfs(self):
        """Count each KFS type on grid"""
        counts = {2: 0, 1: 0, -1: 0}  # R2, R1, Fake
        for r in range(4):
            for c in range(3):
                val = self.grid[r][c]
                if val in counts:
                    counts[val] += 1
        return counts
    
    def validate_grid(self):
        """
        Validate KFS placement - ALL RULES AS WARNINGS for competition flexibility.
        Only count limits are hard errors.
        
        Algorithm should handle ANY config - rules might change at competition!
        """
        counts = self.count_kfs()
        errors = []
        warnings = []
        
        # Count limits - HARD ERRORS (these won't change)
        if counts[2] > 4:
            errors.append(f"R2: {counts[2]}/4 (max)")
        if counts[1] > 3:
            errors.append(f"R1: {counts[1]}/3 (max)")
        if counts[-1] > 1:
            errors.append(f"Fake: {counts[-1]}/1 (max)")
        
        total = counts[2] + counts[1] + counts[-1]
        if total > 8:
            errors.append(f"Total: {total}/8 (max)")
        
        # Rule 3.3.3: R1 on edge - WARNING (rules might change)
        for r in range(4):
            if self.grid[r][1] == 1:
                block_num = r * 3 + 2
                warnings.append(f"R1@{block_num}=middle")
        
        # Rule 3.3.5: Fake on entry - WARNING (rules might change)
        for c in range(3):
            if self.grid[0][c] == -1:
                warnings.append(f"Fake@{c+1}=entry")
        
        # Rule 4.4.14: R2 on entry - WARNING
        entry_r2 = sum(1 for c in range(3) if self.grid[0][c] == 2)
        if counts[2] >= 1 and entry_r2 == 0:
            warnings.append("No R2 on entry")
        
        return errors, warnings
    
    def is_complete(self):
        """Check if grid has 8 KFS - allow any valid config for testing"""
        counts = self.count_kfs()
        return counts[2] + counts[1] + counts[-1] == 8
    
    def cell_to_pixel(self, pos):
        x = self.margin + pos[1] * self.cell_size + self.cell_size // 2
        y = self.margin + pos[0] * self.cell_size + self.cell_size // 2
        return int(x), int(y)
    
    def draw(self):
        heights = get_height_map(self.team_side)
        width = self.margin * 2 + self.cell_size * 3
        height = self.margin * 2 + self.cell_size * 4 + 130
        
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = (30, 30, 30)
        
        # Draw entrance zone
        ez_y = self.margin - 25
        cv2.rectangle(img, (self.margin, ez_y), (self.margin + self.cell_size*3, self.margin-2), (60,60,60), -1)
        cv2.putText(img, "ENTRANCE ZONE", (self.margin + 60, ez_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150,150,150), 1)
        
        for r in range(4):
            for c in range(3):
                x = self.margin + c * self.cell_size
                y = self.margin + r * self.cell_size
                
                h = heights[(r, c)]
                cv2.rectangle(img, (x+2, y+2), (x+self.cell_size-2, y+self.cell_size-2), self.height_colors[h], -1)
                cv2.rectangle(img, (x, y), (x+self.cell_size, y+self.cell_size), (100,100,100), 2)
                
                val = self.grid[r][c]
                if val != 0 and (r, c) not in self.kfs_collected:
                    if self.paused_for_r1 and (r, c) == self.blocking_r1:
                        cv2.circle(img, (x+self.cell_size//2, y+self.cell_size//2), 35, (0,255,255), 3)
                    
                    cv2.circle(img, (x+self.cell_size//2, y+self.cell_size//2), 30, self.colors[val], -1)
                    labels = {2: "R2", 1: "R1", -1: "F"}
                    if val in labels:
                        cv2.putText(img, labels[val], (x+self.cell_size//2-15, y+self.cell_size//2+8),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                    
                    if self.route and val == 2 and (r, c) in self.route.get('kfs_to_pick', []):
                        cx, cy = x+self.cell_size//2, y+self.cell_size//2
                        cv2.line(img, (cx-20, cy-20), (cx+20, cy+20), (0,0,255), 4)
                        cv2.line(img, (cx-20, cy+20), (cx+20, cy-20), (0,0,255), 4)
                
                block_num = r * 3 + c + 1
                cv2.putText(img, str(block_num), (x+5, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150,150,150), 1)
                cv2.putText(img, f"{h}mm", (x+self.cell_size-45, y+self.cell_size-8),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200,200,200), 1)
        
        if self.route:
            path = self.route['path']
            for i in range(len(path) - 1):
                if is_inside_grid(path[i]) and is_inside_grid(path[i+1]):
                    x1, y1 = self.cell_to_pixel(path[i])
                    x2, y2 = self.cell_to_pixel(path[i+1])
                    cv2.line(img, (x1, y1), (x2, y2), (255,150,0), 4)
            
            for i, pos in enumerate(self.route.get('pickup_positions', [])):
                if is_inside_grid(pos):
                    x, y = self.cell_to_pixel(pos)
                    cv2.circle(img, (x, y), 15, (0,255,0), 3)
                    label = "P1" if i == 0 else "P2"
                    if i == 0 and self.route.get('first_from_entrance'):
                        label = "P1*"  # Mark special entrance pickup
                    cv2.putText(img, label, (x-12, y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 2)
            
            if path and is_inside_grid(path[0]):
                ex, ey = self.cell_to_pixel(path[0])
                cv2.circle(img, (ex, ey), 10, (0,255,255), -1)
            if path and is_inside_grid(path[-1]):
                xx, xy = self.cell_to_pixel(path[-1])
                cv2.circle(img, (xx, xy), 10, (255,0,255), -1)
        
        if self.animating and self.robot_pos and is_inside_grid(self.robot_pos):
            rx, ry = self.cell_to_pixel(self.robot_pos)
            cv2.circle(img, (rx, ry), 20, (255,100,0), -1)
            cv2.circle(img, (rx, ry), 20, (255,255,255), 2)
        
        info_y = self.margin + self.cell_size * 4 + 20
        cv2.putText(img, f"Team: {self.team_side}", (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
        
        if self.route:
            cv2.putText(img, f"Cost: {self.route['cost']:.2f}", (150, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
            if self.route.get('first_from_entrance'):
                cv2.putText(img, "[1st from entrance]", (270, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
        
        if self.animating:
            status = "WAIT-R1!" if self.paused_for_r1 else "MOVING"
            cv2.putText(img, status, (10, info_y+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
        
        cv2.putText(img, "Click:cycle R:team C:compute S:start Q:quit", (10, info_y+50), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150,150,150), 1)
        
        # Status and validation
        counts = self.count_kfs()
        errors, warnings = self.validate_grid()
        
        # KFS counts
        count_str = f"R2:{counts[2]}/4 R1:{counts[1]}/3 F:{counts[-1]}/1"
        count_color = (0,200,0) if not errors and self.is_complete() else (200,200,200)
        if errors:
            count_color = (0,0,255)  # Red for errors
        cv2.putText(img, count_str, (10, info_y+70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, count_color, 1)
        
        # Show status: errors (red), warnings (yellow), or ready (green)
        if errors:
            cv2.putText(img, f"ERROR: {errors[0][:35]}", (10, info_y+90), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,0,255), 1)
        elif warnings:
            # Yellow warning but still allow compute
            warn_str = " | ".join(warnings)[:45]
            cv2.putText(img, f"WARN: {warn_str}", (10, info_y+90), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,200,255), 1)
        elif self.is_complete():
            cv2.putText(img, "READY - Press C to compute", (10, info_y+90), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,255,0), 1)
        else:
            remaining = 8 - (counts[2] + counts[1] + counts[-1])
            cv2.putText(img, f"Place {remaining} more KFS", (10, info_y+90), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200,200,200), 1)
        
        legend_y = info_y + 110
        for i, (val, name) in enumerate([(2,"R2"), (1,"R1"), (-1,"F")]):
            cv2.circle(img, (20+i*60, legend_y), 6, self.colors[val], -1)
            cv2.putText(img, name, (32+i*60, legend_y+4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200,200,200), 1)
        
        return img
    
    def on_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            col = (x - self.margin) // self.cell_size
            row = (y - self.margin) // self.cell_size
            
            if 0 <= row < 4 and 0 <= col < 3:
                if self.animating and self.grid[row][col] == 1:
                    self.grid[row][col] = 0
                    print(f"R1 cleared Block {row*3+col+1}!")
                    if self.paused_for_r1 and (row, col) == self.blocking_r1:
                        self.paused_for_r1 = False
                        self.blocking_r1 = None
                else:
                    current = self.grid[row][col]
                    counts = self.count_kfs()
                    
                    # Determine next value based on limits
                    if current == 0:
                        if counts[2] < 4:
                            new_val = 2  # R2
                        elif counts[1] < 3:
                            new_val = 1  # R1
                        elif counts[-1] < 1 and row != 0:  # Fake not on entry
                            new_val = -1
                        else:
                            new_val = 0  # Stay empty
                            print("Max KFS reached!")
                    elif current == 2:
                        if counts[1] < 3:
                            new_val = 1
                        elif counts[-1] < 1 and row != 0:
                            new_val = -1
                        else:
                            new_val = 0
                    elif current == 1:
                        if counts[-1] < 1 and row != 0:
                            new_val = -1
                        else:
                            new_val = 0
                    else:  # current == -1
                        new_val = 0
                    
                    # Special check: Fake not allowed on row 0
                    if new_val == -1 and row == 0:
                        print(f"Fake NOT allowed on Block {row*3+col+1} (entry row)!")
                        new_val = 0
                    
                    self.grid[row][col] = new_val
                    self.route = None
                    self.animating = False
    
    def start_animation(self):
        if not self.route:
            print("Compute first!")
            return
        self.animating = True
        self.path_index = 0
        self.animation_progress = 0.0
        self.robot_pos = self.route['path'][0]
        self.kfs_collected = []
        self.paused_for_r1 = False
        self.blocking_r1 = None
        print("Animation started!")
    
    def update_animation(self):
        if not self.animating or not self.route:
            return
        if self.paused_for_r1:
            return
        
        path = self.route['path']
        if self.path_index >= len(path) - 1:
            print("R2 reached exit!")
            self.animating = False
            return
        
        curr_pos = path[self.path_index]
        next_pos = path[self.path_index + 1]
        
        pickup_positions = self.route.get('pickup_positions', [])
        kfs_order = self.route.get('kfs_order', [])
        for i, p_pos in enumerate(pickup_positions):
            if is_inside_grid(curr_pos) and tuple(curr_pos) == tuple(p_pos) and i < len(kfs_order):
                kfs_pos = kfs_order[i]
                if kfs_pos not in self.kfs_collected:
                    self.kfs_collected.append(kfs_pos)
                    print(f"Picked KFS at Block {kfs_pos[0]*3+kfs_pos[1]+1}")
        
        if is_inside_grid(next_pos) and self.grid[next_pos[0]][next_pos[1]] == 1:
            self.paused_for_r1 = True
            self.blocking_r1 = next_pos
            print(f"Blocked by R1 at Block {next_pos[0]*3+next_pos[1]+1}!")
            return
        
        self.animation_progress += 0.05
        if self.animation_progress >= 1.0:
            self.robot_pos = next_pos
            self.animation_progress = 0.0
            self.path_index += 1
        else:
            if is_inside_grid(curr_pos) and is_inside_grid(next_pos):
                r = curr_pos[0] + (next_pos[0]-curr_pos[0]) * self.animation_progress
                c = curr_pos[1] + (next_pos[1]-curr_pos[1]) * self.animation_progress
                self.robot_pos = (r, c)
    
    def run(self):
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.on_click)
        
        print("=== Forest Nav FINAL ===")
        print("All rules enforced!")
        print()
        
        while True:
            self.update_animation()
            img = self.draw()
            cv2.imshow(self.window_name, img)
            
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                self.team_side = 'BLUE' if self.team_side == 'RED' else 'RED'
                self.route = None
                self.animating = False
                print(f"Team: {self.team_side}")
            elif key == ord('c'):
                heights = get_height_map(self.team_side)
                self.route, error = find_optimal_route(self.grid, heights)
                self.animating = False
                if error:
                    print(f"Error: {error}")
                else:
                    kfs_blocks = [p[0]*3+p[1]+1 for p in self.route['kfs_to_pick']]
                    print(f"\n=== Route Found ===")
                    print(f"KFS: {kfs_blocks}, Cost: {self.route['cost']:.2f}")
                    if self.route.get('first_from_entrance'):
                        print(f"First pickup from ENTRANCE ZONE")
                    if self.route.get('r1_to_clear'):
                        print(f"R1 to clear: {self.route['r1_to_clear']}")
            elif key == ord('s'):
                self.start_animation()
        
        cv2.destroyAllWindows()

if __name__ == "__main__":
    viz = ForestVisualizer()
    viz.run()
