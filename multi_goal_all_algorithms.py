import math
from collections import deque, defaultdict
from heapq import heappush, heappop


# =========================================================
# HELPER FUNCTIONS
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

def extract_goal_visit_paths(full_path, goal_nodes):
    """
    From one complete multi-goal route, extract the first path prefix
    that reaches each destination node.

    Returns:
        [(goal_node, path_prefix_to_that_goal), ...]
    in the order the goals are first visited in the returned route.
    """
    goal_set = set(goal_nodes)
    seen_goals = set()
    goal_paths = []
    for index, node in enumerate(full_path):
        if node in goal_set and node not in seen_goals:
            seen_goals.add(node)
            goal_paths.append((node, full_path[:index + 1]))
    return goal_paths

def min_distance_to_any_goal(node, goal_nodes, node_positions):
    """Heuristic distance from node to the nearest destination node."""
    if not goal_nodes:
        return 0
    return min(
        euclidean_distance(node, goal, node_positions)
        for goal in goal_nodes
    )


def calculate_path_cost(path, graph):
    """Calculate the total edge cost of a path."""
    if path is None or len(path) < 2:
        return 0
    total_cost = 0
    for current_node, next_node in zip(path, path[1:]):
        for neighbor, edge_cost in graph.get(current_node, []):
            if neighbor == next_node:
                total_cost += edge_cost
                break
    return total_cost


def finalize_found_goal_paths(goal_paths, nodes_created, graph):
    """
    Convert collected multi-goal results into the same 5-value format
    expected by search.py:
    (goal_reached, nodes_created, path, total_cost, goal_paths)
    """
    if not goal_paths:
        return None, nodes_created, [], 0, []
    total_cost = sum(calculate_path_cost(path, graph) for _, path in goal_paths)
    last_found_goal = goal_paths[-1][0]
    return last_found_goal, nodes_created, [], total_cost, goal_paths

# =========================================================
# 1. Multi-Goals BFS
# =========================================================
def bfs_all_goals(start_node, goal_nodes, graph, debug=False):
    """
    Multi-goal BFS that records the first BFS path found to each
    reachable destination node during a single search run.
    """
    goal_nodes = set(goal_nodes)
    frontier = deque([(start_node, [start_node])])
    explored = set()
    nodes_created = 1
    found_goals = set()
    goal_paths = []
    while frontier:
        current_node, current_path = frontier.popleft()

        # Record the first path found to this goal, then keep searching
        if current_node in goal_nodes and current_node not in found_goals:
            found_goals.add(current_node)
            goal_paths.append((current_node, current_path.copy()))
            if len(found_goals) == len(goal_nodes):
                break
        if current_node in explored:
            continue
        explored.add(current_node)
        frontier_states = {node for node, _ in frontier}
        for neighbor_node, _ in sorted(graph.get(current_node, []), key=lambda item: item[0]):
            if neighbor_node not in explored and neighbor_node not in frontier_states:
                frontier.append((neighbor_node, current_path + [neighbor_node]))
                nodes_created += 1
        if debug:
            print(f"[MBFS] Current={current_node}, FoundGoals={sorted(found_goals)}")
    return finalize_found_goal_paths(goal_paths, nodes_created, graph)


# =========================================================
# 2. Multi-Goals DFS
# =========================================================
def dfs_all_goals(start_node, goal_nodes, graph, debug=False):
    """
    Multi-goal DFS that records the first DFS path found to each
    reachable destination node during a single search run.
    """
    goal_nodes = set(goal_nodes)
    frontier = [(start_node, [start_node])]
    explored = set()
    nodes_created = 1
    found_goals = set()
    goal_paths = []
    while frontier:
        current_node, current_path = frontier.pop()
        
        # Record the first path found to this goal, then keep searching
        if current_node in goal_nodes and current_node not in found_goals:
            found_goals.add(current_node)
            goal_paths.append((current_node, current_path.copy()))

            if len(found_goals) == len(goal_nodes):
                break
        if current_node in explored:
            continue
        explored.add(current_node)
        frontier_states = {node for node, _ in frontier}

        # Reverse sort before push so smaller IDs are expanded first
        for neighbor_node, _ in sorted(graph.get(current_node, []), key=lambda item: item[0], reverse=True):
            if neighbor_node not in explored and neighbor_node not in frontier_states:
                frontier.append((neighbor_node, current_path + [neighbor_node]))
                nodes_created += 1
        if debug:
            print(f"[MDFS] Current={current_node}, FoundGoals={sorted(found_goals)}")
    return finalize_found_goal_paths(goal_paths, nodes_created, graph)

# =========================================================
# 3. Multi-Goals GBFS
# =========================================================
def gbfs_all_goals(start_node, goal_nodes, graph, node_positions, debug=False):
    """
    Multi-goal GBFS that records the first GBFS path found to each
    reachable destination node during a single search run.
    """
    goal_nodes = set(goal_nodes)
    frontier = []
    explored = set()
    nodes_created = 1
    insertion_order = 0
    found_goals = set()
    goal_paths = []
    start_h = min_distance_to_any_goal(start_node, goal_nodes, node_positions)
    heappush(frontier, (start_h, start_node, insertion_order, [start_node]))
    
    while frontier:
        _, current_node, _, current_path = heappop(frontier)

        # Record the first path found to this goal, then keep searching
        if current_node in goal_nodes and current_node not in found_goals:
            found_goals.add(current_node)
            goal_paths.append((current_node, current_path.copy()))
            if len(found_goals) == len(goal_nodes):
                break
        if current_node in explored:
            continue
        explored.add(current_node)
        frontier_states = {node for _, node, _, _ in frontier}
        for neighbor_node, _ in sorted(graph.get(current_node, []), key=lambda item: item[0]):
            if neighbor_node not in explored and neighbor_node not in frontier_states:
                insertion_order += 1
                neighbor_h = min_distance_to_any_goal(neighbor_node, goal_nodes, node_positions)
                heappush(frontier, (neighbor_h, neighbor_node, insertion_order, current_path + [neighbor_node]))
                nodes_created += 1
        if debug:
            print(f"[MGBFS] Current={current_node}, FoundGoals={sorted(found_goals)}")
    return finalize_found_goal_paths(goal_paths, nodes_created, graph)


# =========================================================
# 4. Multi-Goals A*
# =========================================================
def a_star_all_goals(start_node, goal_nodes, graph, node_positions, debug=False):
    """
    Multi-goal A* that records the first A* path found to each
    reachable destination node during a single search run.
    """
    goal_nodes = set(goal_nodes)
    frontier = []
    best_g = {start_node: 0}
    nodes_created = 1
    found_goals = set()
    goal_paths = []
    start_h = min_distance_to_any_goal(start_node, goal_nodes, node_positions)
    heappush(frontier, (start_h, start_node, 0, [start_node]))
    
    while frontier:
        current_f, current_node, current_g, current_path = heappop(frontier)

        # Skip stale entries.
        # A recorded goal path should only come from the cheapest known route to that node.
        if current_g > best_g.get(current_node, float("inf")):
            continue

        # Record the first path found to this goal, then keep searching
        if current_node in goal_nodes and current_node not in found_goals:
            found_goals.add(current_node)
            goal_paths.append((current_node, current_path.copy()))
            if len(found_goals) == len(goal_nodes):
                break
        for neighbor_node, edge_cost in sorted(graph.get(current_node, []), key=lambda item: item[0]):
            new_g = current_g + edge_cost
            if new_g < best_g.get(neighbor_node, float("inf")):
                best_g[neighbor_node] = new_g
                new_h = min_distance_to_any_goal(neighbor_node, goal_nodes, node_positions)
                new_f = new_g + new_h
                heappush(frontier, (new_f, neighbor_node, new_g, current_path + [neighbor_node]))
                nodes_created += 1
        if debug:
            print(f"[MAS] Current={current_node}, FoundGoals={sorted(found_goals)}, g={current_g}, f={current_f}")
    return finalize_found_goal_paths(goal_paths, nodes_created, graph)


# =========================================================
# 5. CUS1 = UNIFORM-COST SEARCH (LEAST COST)
# =========================================================
def cus1_bidirectional_all_goals(start_node, goal_nodes, graph, debug=False):
    """
    Multi-goal CUS1 implemented as bidirectional search.

    One forward BFS is shared by all goals, while each destination node keeps
    its own backward BFS on the reversed graph. A goal path is recorded when
    the forward search and that goal's backward search meet.
    """
    goal_nodes = set(goal_nodes)
    reverse_edges = defaultdict(list)
    for src, neighbors in graph.items():
        for dest, edge_cost in neighbors:
            reverse_edges[dest].append((src, edge_cost))
    for node in reverse_edges:
        reverse_edges[node].sort(key=lambda item: item[0])

    # Forward search from the origin is shared by all destination nodes.
    f_queue = deque([start_node])
    f_visited = {start_node}
    f_paths = {start_node: [start_node]}
    found_goals = set()
    goal_paths = []
    if start_node in goal_nodes:
        found_goals.add(start_node)
        goal_paths.append((start_node, [start_node]))

    # Each unfinished goal keeps its own backward search on the reversed graph.
    b_searches = {}
    for goal_node in sorted(goal_nodes):
        if goal_node in found_goals:
            continue
        b_searches[goal_node] = {
            "queue": deque([goal_node]),
            "visited": {goal_node},
            "paths": {goal_node: [goal_node]}
        }
    nodes_created = 1 + len(b_searches)

    def record_goal(goal_node, path):
        # Store the first shortest-move path found for this goal, then stop expanding that goal's backward search.
        if goal_node not in found_goals:
            found_goals.add(goal_node)
            goal_paths.append((goal_node, path.copy()))
            b_searches.pop(goal_node, None)

    def expand_forward_layer():
        nonlocal nodes_created
        layer_size = len(f_queue)
        for _ in range(layer_size):
            current_node = f_queue.popleft()
            current_path = f_paths[current_node]

            # A direct forward arrival at a goal is already a shortest-move path.
            if current_node in goal_nodes and current_node not in found_goals:
                record_goal(current_node, current_path)
                if len(found_goals) == len(goal_nodes):
                    return
            for neighbor_node, _ in sorted(graph.get(current_node, []), key=lambda item: item[0]):
                if neighbor_node in f_visited:
                    continue
                f_visited.add(neighbor_node)
                new_path = current_path + [neighbor_node]
                f_paths[neighbor_node] = new_path
                f_queue.append(neighbor_node)
                nodes_created += 1

                # Check the new forward node against every unfinished backward search.
                for goal_node in list(b_searches.keys()):
                    if neighbor_node in b_searches[goal_node]["paths"]:
                        backward_path = b_searches[goal_node]["paths"][neighbor_node]
                        full_path = new_path + backward_path[1:]
                        record_goal(goal_node, full_path)
                if len(found_goals) == len(goal_nodes):
                    return
            if debug:
                print(f"[MCUS1-F] Current={current_node}, FoundGoals={sorted(found_goals)}")

    def expand_backward_layer(goal_node):
        nonlocal nodes_created
        data = b_searches.get(goal_node)
        if data is None:
            return
        layer_size = len(data["queue"])
        for _ in range(layer_size):
            current_node = data["queue"].popleft()
            current_path = data["paths"][current_node]
            for predecessor_node, _ in reverse_edges.get(current_node, []):
                if predecessor_node in data["visited"]:
                    continue
                data["visited"].add(predecessor_node)
                new_path = [predecessor_node] + current_path
                data["paths"][predecessor_node] = new_path
                data["queue"].append(predecessor_node)
                nodes_created += 1

                # A meeting point joins the forward prefix with this goal's backward suffix.
                if predecessor_node in f_paths:
                    full_path = f_paths[predecessor_node] + new_path[1:]
                    record_goal(goal_node, full_path)
                    return
            if debug and goal_node in b_searches:
                print(f"[MCUS1-B] Goal={goal_node}, Current={current_node}")
                
    while f_queue and len(found_goals) < len(goal_nodes):
        expand_forward_layer()
        if len(found_goals) == len(goal_nodes):
            break
        for goal_node in list(b_searches.keys()):
            expand_backward_layer(goal_node)
            if len(found_goals) == len(goal_nodes):
                break
    return finalize_found_goal_paths(goal_paths, nodes_created, graph)


# =========================================================
# 6. CUS2 = IDA* (LEAST COST)
# =========================================================
def cus2_ida_star_all_goals(start_node, goal_nodes, graph, node_positions, debug=False):
    """
    Multi-goal CUS2 implemented as IDA*.

    Each IDA* iteration explores all paths with f <= current_bound and updates
    the cheapest path found so far for every destination node reached in that
    iteration.
    """
    goal_nodes = set(goal_nodes)
    sorted_graph = {
        node: sorted(neighbors, key=lambda item: item[0])
        for node, neighbors in graph.items()
    }
    found_goal_costs = {}
    found_goal_paths = {}
    goal_discovery_order = []
    total_nodes_created = 1
    current_bound = min_distance_to_any_goal(start_node, goal_nodes, node_positions)

    def record_goal(goal_node, path_cost, path):
        # A goal can be reached more than once across different bounds, so keep
        # the cheapest path seen so far for that destination node.
        previous_cost = found_goal_costs.get(goal_node, float("inf"))
        if goal_node not in found_goal_paths:
            goal_discovery_order.append(goal_node)
        if path_cost < previous_cost:
            found_goal_costs[goal_node] = path_cost
            found_goal_paths[goal_node] = path.copy()

    def bounded_dfs(current_node, current_path, path_cost_so_far, current_bound, best_g_this_iteration):
        nonlocal total_nodes_created
        heuristic_cost = min_distance_to_any_goal(current_node, goal_nodes, node_positions)
        estimated_total_cost = path_cost_so_far + heuristic_cost
        if debug:
            print(
                f"[MCUS2] Current={current_node}, FoundGoals={sorted(found_goal_paths)}, "
                f"g={path_cost_so_far:.2f}, h={heuristic_cost:.2f}, "
                f"f={estimated_total_cost:.2f}, bound={current_bound:.2f}"
            )

        # Standard IDA* pruning: paths above the current bound are deferred to a later iteration.
        if estimated_total_cost > current_bound:
            return estimated_total_cost

        # Keep only the cheapest path to each node within the current bound-limited pass.
        if path_cost_so_far > best_g_this_iteration.get(current_node, float("inf")):
            return float("inf")
        if current_node in goal_nodes:
            # Reaching one goal does not end the search, because this bound may still
            # contain cheaper paths to other goals.
            record_goal(current_node, path_cost_so_far, current_path)
        smallest_exceeded_bound = float("inf")
        for neighbor_node, edge_cost in sorted_graph.get(current_node, []):
            if neighbor_node in current_path:
                continue
            new_cost = path_cost_so_far + edge_cost
            if new_cost >= best_g_this_iteration.get(neighbor_node, float("inf")):
                continue
            best_g_this_iteration[neighbor_node] = new_cost
            current_path.append(neighbor_node)
            total_nodes_created += 1
            returned_bound = bounded_dfs(
                neighbor_node,
                current_path,
                new_cost,
                current_bound,
                best_g_this_iteration
            )
            smallest_exceeded_bound = min(smallest_exceeded_bound, returned_bound)
            current_path.pop()
        return smallest_exceeded_bound

    while True:
        best_g_this_iteration = {start_node: 0}
        next_bound = bounded_dfs(
            start_node,
            [start_node],
            0,
            current_bound,
            best_g_this_iteration
        )
        
        # After a full iteration, every goal found within this bound has its
        # cheapest path for this bound already recorded.
        if len(found_goal_paths) == len(goal_nodes):
            break
        if next_bound == float("inf"):
            break
        current_bound = next_bound
    goal_paths = [(goal_node, found_goal_paths[goal_node]) for goal_node in goal_discovery_order]
    return finalize_found_goal_paths(goal_paths, total_nodes_created, graph)