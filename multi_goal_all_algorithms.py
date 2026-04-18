import math
from collections import deque
from heapq import heappush, heappop


# =========================================================
# HELPERS
# =========================================================

def euclidean_distance(node_a, node_b, node_positions):
    """
    Computes the straight-line (Euclidean) distance between two nodes
    using their (x, y) coordinates from node_positions.
    Returns 0 if either node has no recorded position (safe fallback).
    """
    if node_a not in node_positions or node_b not in node_positions:
        return 0  # Guard: avoid KeyError for unknown nodes
    x1, y1 = node_positions[node_a]
    x2, y2 = node_positions[node_b]
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def nearest_remaining_goal_heuristic(current_node, visited_goals, all_goals, node_positions):
    """
    Admissible lower bound heuristic for multi-goal search.

    Returns the Euclidean distance from the current node to the
    nearest *unvisited* goal. This never overestimates the true
    remaining cost, so it is admissible for A* and IDA*.

    Returns 0 when all goals have already been visited (terminal state).
    """
    remaining_goals = set(all_goals) - set(visited_goals)
    if not remaining_goals:
        return 0  # No goals left — we're done, heuristic cost is 0

    # Pick the closest unvisited goal as the optimistic estimate
    return min(
        euclidean_distance(current_node, goal, node_positions)
        for goal in remaining_goals
    )


def reconstruct_path(parent_map, goal_state):
    """
    Traces back through parent_map from the goal state to the start,
    rebuilding the sequence of node IDs that form the solution path.

    parent_map: dict mapping each state -> its predecessor state
    goal_state: the terminal (node, visited_goals) tuple

    Returns a list of node IDs from start to goal (inclusive).
    """
    path = [goal_state[0]]       # Start the path with the goal node
    current_state = goal_state

    # Walk backwards through the parent chain until we reach the root
    while current_state in parent_map:
        current_state = parent_map[current_state]
        path.append(current_state[0])  # Prepend the parent node

    path.reverse()  # Flip from goal→start to start→goal
    return path


def make_start_state(start_node, goal_nodes):
    """
    Creates the initial search state as a (node, visited_goals) tuple.

    If the start node itself is one of the goals, it is pre-marked as visited
    so it doesn't need to be revisited later. visited_goals is a frozenset
    so it can be used as a dictionary key or set member (hashable).
    """
    # Pre-visit the start node if it is a goal
    visited_goals = frozenset([start_node]) if start_node in goal_nodes else frozenset()
    return (start_node, visited_goals)


# =========================================================
# 1. BFS (LEAST MOVES, NOT LEAST COST)
# =========================================================

def bfs_all_goals(start_node, goal_nodes, graph, debug=False):
    """
    Breadth-First Search adapted for multi-goal traversal.

    Strategy:
    - Explores states level by level (FIFO queue).
    - Guarantees the path with the fewest edges (moves) to visit all goals.
    - Does NOT guarantee the minimum total edge weight in weighted graphs.

    State space: (current_node, frozenset_of_visited_goals)
    Returns: (final_node, nodes_created, path, total_cost)
    """
    goal_nodes = set(goal_nodes)
    start_state = make_start_state(start_node, goal_nodes)

    # Frontier holds: (state, path_so_far, accumulated_cost)
    frontier = deque([(start_state, [start_node], 0)])
    visited = {start_state}   # Tracks expanded states to avoid re-expansion
    nodes_created = 1         # Count the start node

    while frontier:
        # Pop from the left — BFS processes nodes in arrival order (FIFO)
        (current_node, visited_goals), current_path, current_cost = frontier.popleft()

        # Goal check: all goals must be in the visited set
        if visited_goals == goal_nodes:
            return current_node, nodes_created, current_path, current_cost

        # Expand neighbors in alphabetical/numeric order for determinism
        for neighbor_node, edge_cost in sorted(graph.get(current_node, []), key=lambda item: item[0]):
            new_visited_goals = visited_goals

            # If this neighbor is an unvisited goal, mark it as visited
            if neighbor_node in goal_nodes:
                new_visited_goals = frozenset(set(visited_goals) | {neighbor_node})

            new_state = (neighbor_node, new_visited_goals)

            # Only add states we haven't seen before (avoids infinite loops)
            if new_state not in visited:
                visited.add(new_state)
                frontier.append((new_state, current_path + [neighbor_node], current_cost + edge_cost))
                nodes_created += 1

        if debug:
            print(f"[BFS] Current={current_node}, VisitedGoals={sorted(visited_goals)}, Cost={current_cost}")

    # Frontier exhausted — no complete solution found
    return None, nodes_created, [], 0


# =========================================================
# 2. DFS (BRANCH-AND-BOUND, LEAST COST)
# =========================================================

def dfs_all_goals_optimal(start_node, goal_nodes, graph, debug=False):
    """
    Depth-First Search with Branch-and-Bound for multi-goal traversal.

    Strategy:
    - Explores depth-first (recursively), backtracking after each branch.
    - Prunes any branch whose accumulated cost already meets or exceeds
      the best solution found so far (branch-and-bound).
    - Tries all valid paths, so it guarantees the minimum-cost route.
    - Cycle detection via path membership check prevents infinite loops.

    Returns: (final_node, nodes_created, path, total_cost)
    """
    goal_nodes = set(goal_nodes)
    start_state = make_start_state(start_node, goal_nodes)

    # Shared mutable record of the best complete solution found so far
    best_solution = {
        "cost": float("inf"),  # Start with worst-case so any solution improves it
        "path": None,
        "goal": None
    }

    nodes_created = 1  # Count the start node

    def dfs_recursive(current_node, visited_goals, current_path, current_cost):
        nonlocal nodes_created

        # Pruning: skip this branch if it can't beat the best known solution
        if current_cost >= best_solution["cost"]:
            return

        # Terminal check: all goals visited — update best if this is cheaper
        if visited_goals == goal_nodes:
            best_solution["cost"] = current_cost
            best_solution["path"] = current_path.copy()
            best_solution["goal"] = current_node
            return

        # Sort neighbors in reverse so that smaller-id nodes are explored first
        # (reverse sort + stack = ascending order of expansion)
        for neighbor_node, edge_cost in sorted(
            graph.get(current_node, []), key=lambda item: item[0], reverse=True
        ):
            new_cost = current_cost + edge_cost

            # Cycle guard: skip this neighbor if it is already on the current path
            if neighbor_node in current_path:
                continue

            new_visited_goals = visited_goals
            if neighbor_node in goal_nodes:
                new_visited_goals = frozenset(set(visited_goals) | {neighbor_node})

            nodes_created += 1
            current_path.append(neighbor_node)          # Go deeper
            dfs_recursive(neighbor_node, new_visited_goals, current_path, new_cost)
            current_path.pop()                          # Backtrack

    # Kick off the recursion from the start
    dfs_recursive(start_node, start_state[1], [start_node], 0)

    if best_solution["path"] is not None:
        return best_solution["goal"], nodes_created, best_solution["path"], best_solution["cost"]

    return None, nodes_created, [], 0


# =========================================================
# 3. GBFS (HEURISTIC-ORDERED BRANCH-AND-BOUND, LEAST COST)
# =========================================================

def gbfs_all_goals_optimal(start_node, goal_nodes, graph, node_positions, debug=False):
    """
    Greedy Best-First Search with Branch-and-Bound for multi-goal traversal.

    Strategy:
    - Like DFS-optimal, but neighbors are sorted by heuristic value first,
      so branches that look promising (closer to remaining goals) are
      explored before less promising ones.
    - Still exhaustive — every non-pruned branch is fully explored.
    - The heuristic ordering often finds good solutions earlier, leading
      to stronger pruning and fewer total nodes expanded.

    Returns: (final_node, nodes_created, path, total_cost)
    """
    goal_nodes = set(goal_nodes)
    start_state = make_start_state(start_node, goal_nodes)

    best_solution = {
        "cost": float("inf"),
        "path": None,
        "goal": None
    }

    nodes_created = 1

    def gbfs_recursive(current_node, visited_goals, current_path, current_cost):
        nonlocal nodes_created

        # Prune: branch cannot improve on the best known cost
        if current_cost >= best_solution["cost"]:
            return

        # All goals visited — record if this is the best solution so far
        if visited_goals == goal_nodes:
            best_solution["cost"] = current_cost
            best_solution["path"] = current_path.copy()
            best_solution["goal"] = current_node
            return

        # Build neighbor list with heuristic values for ordering
        neighbors = []
        for neighbor_node, edge_cost in graph.get(current_node, []):
            # Skip nodes already on the current path (cycle prevention)
            if neighbor_node in current_path:
                continue

            new_visited_goals = visited_goals
            if neighbor_node in goal_nodes:
                new_visited_goals = frozenset(set(visited_goals) | {neighbor_node})

            # Compute heuristic: distance to nearest remaining goal from neighbor
            h_value = nearest_remaining_goal_heuristic(
                neighbor_node, new_visited_goals, goal_nodes, node_positions
            )
            neighbors.append((h_value, neighbor_node, edge_cost, new_visited_goals))

        # Sort by heuristic first, then node ID as a tiebreaker
        neighbors.sort(key=lambda item: (item[0], item[1]))

        for _, neighbor_node, edge_cost, new_visited_goals in neighbors:
            new_cost = current_cost + edge_cost
            nodes_created += 1
            current_path.append(neighbor_node)
            gbfs_recursive(neighbor_node, new_visited_goals, current_path, new_cost)
            current_path.pop()  # Backtrack after exploring this branch

    gbfs_recursive(start_node, start_state[1], [start_node], 0)

    if best_solution["path"] is not None:
        return best_solution["goal"], nodes_created, best_solution["path"], best_solution["cost"]

    return None, nodes_created, [], 0


# =========================================================
# 4. A* (LEAST COST)
# =========================================================

def a_star_all_goals(start_node, goal_nodes, graph, node_positions, debug=False):
    """
    A* Search adapted for multi-goal traversal.

    Strategy:
    - Uses a min-heap (priority queue) ordered by f = g + h, where:
        g = actual cost from start to current state
        h = admissible heuristic (nearest remaining goal distance)
    - Guarantees the least-cost path to visit all goals, provided h is admissible.
    - Re-expansion is avoided by skipping states whose recorded g_cost has
      already been improved by a previously processed path.

    State: (current_node, frozenset_of_visited_goals)
    Returns: (final_node, nodes_created, path, total_cost)
    """
    goal_nodes = set(goal_nodes)
    start_state = make_start_state(start_node, goal_nodes)

    frontier = []           # Min-heap: entries are (f, node, g, state)
    parent_map = {}         # Maps each state to its predecessor state (for path reconstruction)
    g_cost = {start_state: 0}   # Best known g cost for each state
    nodes_created = 1

    # Compute initial heuristic for the start node
    start_h = nearest_remaining_goal_heuristic(
        start_node, start_state[1], goal_nodes, node_positions
    )
    heappush(frontier, (start_h, start_node, 0, start_state))

    while frontier:
        # Pop the state with the lowest f = g + h
        current_f, _, current_g, current_state = heappop(frontier)
        current_node, visited_goals = current_state

        # Skip stale entries — a cheaper path to this state was already found
        if current_g > g_cost.get(current_state, float("inf")):
            continue

        # Goal check: all goals visited
        if visited_goals == goal_nodes:
            return current_node, nodes_created, reconstruct_path(parent_map, current_state), current_g

        # Expand each neighbor
        for neighbor_node, edge_cost in sorted(graph.get(current_node, []), key=lambda item: item[0]):
            new_visited_goals = visited_goals
            if neighbor_node in goal_nodes:
                new_visited_goals = frozenset(set(visited_goals) | {neighbor_node})

            new_state = (neighbor_node, new_visited_goals)
            new_g = current_g + edge_cost

            # Only add to frontier if this is a cheaper path to this state
            if new_g < g_cost.get(new_state, float("inf")):
                g_cost[new_state] = new_g
                parent_map[new_state] = current_state  # Record how we got here

                # Compute heuristic for neighbor and push with updated f value
                new_h = nearest_remaining_goal_heuristic(
                    neighbor_node, new_visited_goals, goal_nodes, node_positions
                )
                new_f = new_g + new_h
                heappush(frontier, (new_f, neighbor_node, new_g, new_state))
                nodes_created += 1

        if debug:
            print(f"[A*] Current={current_node}, VisitedGoals={sorted(visited_goals)}, g={current_g}, f={current_f}")

    return None, nodes_created, [], 0


# =========================================================
# 5. CUS1 = UNIFORM-COST SEARCH (LEAST COST)
# =========================================================

def cus1_uniform_cost_all_goals(start_node, goal_nodes, graph, debug=False):
    """
    Uniform-Cost Search (UCS) adapted for multi-goal traversal.

    Strategy:
    - Like A* but with h = 0 (no heuristic). Expands states purely by
      accumulated path cost (g), making it equivalent to Dijkstra's algorithm.
    - Guarantees the minimum-cost path to visit all goals.
    - An insertion_order counter breaks ties so the heap is stable and
      consistent when costs are equal (avoids comparing frozensets).

    State: (current_node, frozenset_of_visited_goals)
    Returns: (final_node, nodes_created, path, total_cost)
    """
    goal_nodes = set(goal_nodes)
    start_state = make_start_state(start_node, goal_nodes)

    frontier = []                       # Min-heap: (cost, node, insertion_order, state)
    parent_map = {}
    best_cost = {start_state: 0}       # Best known cost for each state
    nodes_created = 1
    insertion_order = 0                 # Monotonically increasing tie-breaker

    heappush(frontier, (0, start_node, insertion_order, start_state))

    while frontier:
        current_cost, _, _, current_state = heappop(frontier)
        current_node, visited_goals = current_state

        # Skip if a cheaper path to this state has already been processed
        if current_cost > best_cost.get(current_state, float("inf")):
            continue

        # Goal check: all goals visited
        if visited_goals == goal_nodes:
            return current_node, nodes_created, reconstruct_path(parent_map, current_state), current_cost

        for neighbor_node, edge_cost in sorted(graph.get(current_node, []), key=lambda item: item[0]):
            new_visited_goals = visited_goals
            if neighbor_node in goal_nodes:
                new_visited_goals = frozenset(set(visited_goals) | {neighbor_node})

            new_state = (neighbor_node, new_visited_goals)
            new_cost = current_cost + edge_cost

            # Relax: push if this path to the new state is strictly cheaper
            if new_cost < best_cost.get(new_state, float("inf")):
                best_cost[new_state] = new_cost
                parent_map[new_state] = current_state
                insertion_order += 1   # Ensure stable ordering within equal-cost states
                heappush(frontier, (new_cost, neighbor_node, insertion_order, new_state))
                nodes_created += 1

        if debug:
            print(f"[CUS1-UCS] Current={current_node}, VisitedGoals={sorted(visited_goals)}, Cost={current_cost}")

    return None, nodes_created, [], 0


# =========================================================
# 6. CUS2 = IDA* (LEAST COST)
# =========================================================

def ida_star_all_goals(start_node, goal_nodes, graph, node_positions, debug=False):
    """
    Iterative Deepening A* (IDA*) adapted for multi-goal traversal.

    Strategy:
    - Performs depth-first searches with a cost threshold (bound).
    - Each iteration raises the bound to the smallest f-value that
      exceeded the previous bound, until a solution is found.
    - Memory-efficient: only stores the current path on the call stack,
      not the full frontier. Suitable for large search spaces.
    - Guarantees the least-cost solution if the heuristic is admissible.

    Returns: (final_node, nodes_created, path, total_cost)
    """
    goal_nodes = set(goal_nodes)
    start_state = make_start_state(start_node, goal_nodes)

    # Pre-sort adjacency lists once so each recursive call doesn't re-sort
    sorted_graph = {
        node: sorted(neighbors, key=lambda item: item[0])
        for node, neighbors in graph.items()
    }

    # The path is stored as a list of (node, visited_goals) tuples
    current_path = [start_state]

    # Initial bound = h(start): the optimistic cost to reach all goals
    current_bound = nearest_remaining_goal_heuristic(
        start_node, start_state[1], goal_nodes, node_positions
    )
    total_nodes_created = 1

    # Iteratively tighten the bound until a solution is found or space exhausted
    while True:
        best = [None]  # Mutable container so ida_dfs can update it in-place

        # Run a bounded DFS; returns the next threshold if no solution found
        _, next_bound, total_nodes_created = ida_dfs(
            current_path=current_path,
            path_cost_so_far=0,
            current_bound=current_bound,
            goal_nodes=goal_nodes,
            sorted_graph=sorted_graph,
            node_positions=node_positions,
            total_nodes_created=total_nodes_created,
            best=best,
            debug=debug
        )

        if best[0] is not None:
            # A complete solution was found within the current bound
            best_cost, best_path = best[0]
            node_path = [node for node, _ in best_path]
            return node_path[-1], total_nodes_created, node_path, best_cost

        if next_bound == float("inf"):
            # No solution exists in any bound — graph is disconnected or unsolvable
            return None, total_nodes_created, [], 0

        # Raise the bound to the smallest f-value that was pruned this iteration
        current_bound = next_bound


def ida_dfs(current_path, path_cost_so_far, current_bound, goal_nodes,
            sorted_graph, node_positions, total_nodes_created, best, debug=False):
    """
    Recursive DFS helper for IDA*.

    Explores nodes depth-first, pruning any branch whose f = g + h
    exceeds current_bound. Tracks the minimum exceeded f-value so the
    caller knows what threshold to use in the next iteration.

    current_path  : list of (node, visited_goals) tuples on the current path
    path_cost_so_far : g-cost accumulated along current_path
    current_bound : maximum allowed f = g + h for this iteration
    best          : single-element list holding the best complete solution found
                    (allows mutation from nested calls without nonlocal)

    Returns: (found, next_minimum_bound, total_nodes_created)
    """
    current_node, visited_goals = current_path[-1]

    # Compute f = g + h for the current state
    h_value = nearest_remaining_goal_heuristic(
        current_node, visited_goals, goal_nodes, node_positions
    )
    f_value = path_cost_so_far + h_value

    # Prune: f exceeds the current threshold — return f as a candidate next bound
    if f_value > current_bound:
        return False, f_value, total_nodes_created

    # All goals visited — candidate solution found
    if visited_goals == goal_nodes:
        candidate_path = current_path.copy()

        if best[0] is None:
            # First solution found this iteration
            best[0] = (path_cost_so_far, candidate_path)
        else:
            best_cost, best_path = best[0]
            candidate_goal = candidate_path[-1][0]
            best_goal = best_path[-1][0]

            # Keep this solution if it's cheaper, or ties on cost but ends
            # at a node with a smaller ID (deterministic tiebreaking)
            if (
                path_cost_so_far < best_cost or
                (path_cost_so_far == best_cost and candidate_goal < best_goal)
            ):
                best[0] = (path_cost_so_far, candidate_path)

        # Return inf so caller doesn't raise the bound because of this branch
        return False, float("inf"), total_nodes_created

    # Track the smallest f-value that exceeded the bound across all children
    smallest_exceeded_bound = float("inf")

    # Extract just the node IDs on the current path for cycle detection
    current_nodes_only = [node for node, _ in current_path]

    for neighbor_node, edge_cost in sorted_graph.get(current_node, []):
        # Skip nodes already on the path — prevents cycles
        if neighbor_node in current_nodes_only:
            continue

        new_visited_goals = visited_goals
        if neighbor_node in goal_nodes:
            new_visited_goals = frozenset(set(visited_goals) | {neighbor_node})

        # Push this neighbor onto the path and recurse
        current_path.append((neighbor_node, new_visited_goals))
        total_nodes_created += 1

        _, returned_bound, total_nodes_created = ida_dfs(
            current_path=current_path,
            path_cost_so_far=path_cost_so_far + edge_cost,
            current_bound=current_bound,
            goal_nodes=goal_nodes,
            sorted_graph=sorted_graph,
            node_positions=node_positions,
            total_nodes_created=total_nodes_created,
            best=best,
            debug=debug
        )

        # Keep track of the minimum f that exceeded the threshold
        if returned_bound is not None:
            smallest_exceeded_bound = min(smallest_exceeded_bound, returned_bound)

        current_path.pop()  # Backtrack: remove neighbor from the current path

    return False, smallest_exceeded_bound, total_nodes_created