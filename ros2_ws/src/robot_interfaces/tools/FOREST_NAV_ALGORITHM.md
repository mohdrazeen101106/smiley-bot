# Forest Navigation Algorithm - Complete Technical Documentation

## Overview

This document explains the pathfinding algorithm used by R2 to navigate through the Meihua Forest, collect 2 KFS (Kung-Fu Scrolls), and exit successfully.

---

## 1. Grid Layout

```
         ENTRANCE ZONE
              ↓
    ┌─────┬─────┬─────┐
    │  1  │  2  │  3  │  Row 0 (Entry Row)
    ├─────┼─────┼─────┤
    │  4  │  5  │  6  │  Row 1
    ├─────┼─────┼─────┤
    │  7  │  8  │  9  │  Row 2
    ├─────┼─────┼─────┤
    │ 10  │ 11  │ 12  │  Row 3 (Exit Row)
    └─────┴─────┴─────┘
       ↓         ↓
    EXIT 1    EXIT 2
```

### Coordinate System
- Grid: 4 rows × 3 columns
- Block number = row × 3 + column + 1
- Entry zone: Above row 0, robot enters at block 2
- Exits: Blocks 10 (left) and 12 (right)

---

## 2. Block Types & Passability

| Type | Value | Passable? | Notes |
|------|-------|-----------|-------|
| Empty | 0 | ✅ Yes | Always passable |
| R1 KFS | 1 | Conditional | Passable if R1 can reach (see below) |
| R2 KFS | 2 | Conditional | Passable ONLY after collected |
| Fake KFS | -1 | ❌ No | Always blocked |

### R1 Reachability (Critical!)

R1 robot travels on pathways around the forest perimeter. It can only clear R1 KFS on blocks it can physically reach.

```python
def can_r1_reach(pos):
    r, c = pos
    # Row 0 (entry) and Row 3 (exit) - all columns reachable
    if r == 0 or r == 3:
        return True
    # Rows 1 and 2 - only edge columns (0 and 2) reachable
    return c == 0 or c == 2
```

| Block | R1 Can Reach? | Reason |
|-------|---------------|--------|
| 1,2,3 | ✅ Yes | Adjacent to entry pathway |
| 4,6 | ✅ Yes | Adjacent to side pathways |
| **5** | ❌ **No** | Center of row 1, no pathway access |
| 7,9 | ✅ Yes | Adjacent to side pathways |
| **8** | ❌ **No** | Center of row 2, no pathway access |
| 10,11,12 | ✅ Yes | Adjacent to exit pathway |

**Important**: If R1 KFS is placed on block 5 or 8, it becomes a **permanent obstacle** that cannot be cleared.

---

## 3. Core Algorithm: `find_optimal_route()`

### Step 1: Identify R2 KFS Positions
```python
r2_kfs = [(r, c) for r in range(4) for c in range(3) if grid[r][c] == 2]
```

### Step 2: Determine First KFS Candidates

Per **Rule 4.4.14**: "R2 must collect first KFS from blocks 1, 2, or 3"

```python
entry_row_kfs = [k for k in r2_kfs if k[0] == 0]  # Row 0 = blocks 1,2,3

if entry_row_kfs:
    first_candidates = entry_row_kfs  # MUST pick from entry row
else:
    first_candidates = r2_kfs  # No entry KFS - pick any (illegal config)
```

### Step 3: Try All Route Combinations

For each first KFS candidate:
1. **Entry row KFS**: Pick from entrance zone (before entering grid)
2. **Other KFS**: Navigate to adjacent block, then pick

```python
for first_kfs in first_candidates:
    if first_kfs[0] == 0:  # Entry row
        first_pickup = ENTRY  # Special: pickup from entrance zone
        enter_at = ENTRY_BLOCK  # Block 2
    else:
        # Need to enter grid first, then navigate to adjacent position
        pickups = get_normal_pickup_positions(grid, first_kfs)
        # Try each pickup position...
```

### Step 4: BFS Pathfinding

The algorithm uses **Breadth-First Search (BFS)** to find shortest paths.

```python
def bfs_path(grid, start, end, collected_kfs=None):
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
    return None  # No path found
```

**Key**: `collected_kfs` parameter tracks which R2 KFS have been collected - these blocks become passable.

### Step 5: Calculate Path Cost

```python
def calculate_path_cost(path, heights):
    cost = 0
    for i in range(len(path) - 1):
        h1 = heights[path[i]]
        h2 = heights[path[i+1]]
        cost += 1  # Base movement cost
        if h2 > h1:
            cost += (h2 - h1) / 200 * 0.5  # Climbing penalty
    return cost
```

Height map for RED team:
```
400mm  200mm  400mm
200mm  400mm  600mm
400mm  600mm  400mm
200mm  400mm  200mm
```

---

## 4. Route Building: `try_build_route()`

A complete route has 3 segments:

### Segment 1: Entry → First Pickup Position
```python
if pickup1 == ENTRY:
    seg1 = [enter_at]  # Start at block 2 after entrance pickup
else:
    seg1 = bfs_path(grid, ENTRY, pickup1, collected_kfs=set())
```

### Segment 2: First Pickup → Second Pickup
```python
collected_1 = {kfs1}  # First KFS now collected
seg2 = bfs_path(grid, seg1[-1], pickup2, collected_kfs=collected_1)
```

### Segment 3: Second Pickup → Exit
```python
collected_2 = {kfs1, kfs2}  # Both collected
seg3 = bfs_path(grid, pickup2, exit_pos, collected_kfs=collected_2)
```

---

## 5. Fallback Strategies

### Strategy 1: 1-KFS Fallback

When no 2-KFS route exists, try to at least pick 1 KFS and exit.

```python
def try_one_kfs_fallback(grid, heights, r2_kfs, first_candidates):
    for kfs in first_candidates:
        if kfs[0] == 0:  # Entry row
            for exit_pos in EXITS:
                path = bfs_path(grid, ENTRY_BLOCK, exit_pos, collected_kfs={kfs})
                if path:
                    return route
```

### Strategy 2: Drop-to-Clear (Illegal Configs Only)

For configs with R1 on blocks 5 or 8, the algorithm can pick up a blocking R2 KFS, drop it elsewhere, and continue.

```python
def try_drop_to_clear(grid, heights, r2_kfs, first_candidates):
    # 1. Pick first KFS from entry
    # 2. Navigate to blocking R2 KFS
    # 3. Pick up blocker
    # 4. Drop blocker on empty adjacent block
    # 5. Pick second target KFS
    # 6. Exit with 2 KFS
```

---

## 6. Adjacency Rules

### Grid Adjacency
```python
def get_adjacent(pos):
    r, c = pos
    adj = []
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Up, Down, Left, Right
        nr, nc = r + dr, c + dc
        if 0 <= nr < 4 and 0 <= nc < 3:
            adj.append((nr, nc))
    return adj
```

**Important**: Diagonal movement is NOT allowed (Rule FAQ 4.4-o).

### Pickup Rules

1. **First KFS** (from entry row): Picked while in entrance zone
2. **Subsequent KFS**: Must be on adjacent block inside grid

```python
def get_normal_pickup_positions(grid, kfs_pos, collected_kfs=None):
    pickups = []
    for adj in get_adjacent(kfs_pos):
        if is_passable(grid, adj, collected_kfs):
            pickups.append(adj)
    return pickups
```

---

## 7. Algorithm Flowchart

```
┌─────────────────────────────────────┐
│         START                        │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│  Find all R2 KFS positions          │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│  Any R2 in entry row (1,2,3)?       │
└──────────────┬──────────────────────┘
        Yes ↓          ↓ No
┌──────────────┐  ┌────────────────────┐
│ First must   │  │ First can be any   │
│ be from      │  │ R2 KFS (illegal    │
│ entry row    │  │ config)            │
└──────────────┘  └────────────────────┘
               ↓
┌─────────────────────────────────────┐
│  For each first KFS candidate:      │
│    For each second KFS:             │
│      For each exit:                 │
│        Try to build route           │
│        Track best (lowest cost)     │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│  Route found?                       │
└──────────────┬──────────────────────┘
        Yes ↓          ↓ No
┌──────────────┐  ┌────────────────────┐
│ Return best  │  │ Try fallbacks:     │
│ route        │  │ 1. Drop-to-clear   │
│              │  │ 2. 1-KFS fallback  │
└──────────────┘  └────────────────────┘
               ↓
┌─────────────────────────────────────┐
│         END                          │
└─────────────────────────────────────┘
```

---

## 8. Legal vs Illegal Configs

### Legal Config Requirements (per Rules 3.3.3-3.3.5):
- R1 KFS only on blocks 1,2,3,4,6,7,9,10,11,12
- R2 KFS on any vacant block
- Fake KFS not on blocks 1,2,3 (entry row)

### Illegal Configs:
- R1 on block 5 or 8 (unreachable by R1)
- May require drop-to-clear strategy
- Some are completely unsolvable (physical walls)

---

## 9. Performance Summary

| Category | Count | Solve Rate |
|----------|-------|------------|
| Total configs | 138,600 | - |
| Legal configs | 75,600 | **100%** |
| Illegal configs | 63,000 | 98% |
| Using drop-to-clear | 958 | - |
| Unsolvable | 1,292 | 0% (physical walls) |

---

## 10. Key Functions Reference

| Function | Purpose |
|----------|---------|
| `find_optimal_route()` | Main entry point, orchestrates route finding |
| `try_first_kfs_route()` | Tries routes with given first KFS |
| `try_build_route()` | Builds complete path with 3 segments |
| `bfs_path()` | BFS pathfinding between two positions |
| `is_passable()` | Checks if a block can be traversed |
| `can_r1_reach()` | Checks if R1 can clear a block |
| `get_adjacent()` | Gets orthogonally adjacent blocks |
| `get_normal_pickup_positions()` | Finds valid pickup positions |
| `calculate_path_cost()` | Calculates total path cost with height penalty |
| `try_one_kfs_fallback()` | 1-KFS fallback strategy |
| `try_drop_to_clear()` | Drop-to-clear strategy for illegal configs |

---

## 11. Rule References

| Rule | Description |
|------|-------------|
| 3.3.3 | R1 KFS placed on blocks adjacent to pathway |
| 3.3.4 | R2 KFS on any vacant block |
| 3.3.5 | Fake KFS not on entrance blocks (1,2,3) |
| 4.4.14 | First KFS pickup from blocks 1,2,3 |
| 4.4-r(a) | First KFS from entrance zone |
| 4.4-r(b) | Subsequent KFS from adjacent blocks |
| 4.4-o | Adjacent = orthogonal only (no diagonals) |
| 4.4-p | R2 occupies one block at a time |
| 4.4.18 | Dropped KFS outside forest cannot be reused |
| 4.4.19 | R2 can pick up dropped KFS inside forest |
