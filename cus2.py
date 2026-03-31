import math
from utils import heuristic

# Custom Informed Search Algorithm (CUS2) - Iterative Deepening A* (IDA*) Search with Tie-Breaking Rule:
# Repeatedly performs depth-first search using an increasing f-cost bound, where f(n) = g(n) + h(n), until a path to a goal node is determined
# If all else equal:
# 1. prefer lower total path cost
# 2. if equal cost, prefer smaller goal node number
# 3. if equal cost and same goal node number, prefer the path found first
def ida_star_search(start_node, goal_nodes, graph, node_positions, debug=False):
    goal_set = set(goal_nodes)
    sorted_graph = {node: sorted(neighbors, key=lambda item: item[0]) for node, neighbors in graph.items()}
    current_path = [start_node]
    current_bound = heuristic(start_node, goal_set, node_positions)
    total_nodes_created = 1
    while True:
        best = [None]
        _, next_bound, total_nodes_created = f_bounded_dfs(current_path, 0, current_bound, goal_set, sorted_graph, node_positions, total_nodes_created, best, debug)
        if best[0] is not None:
            _, path = best[0]
            return path[-1], total_nodes_created, path
        if next_bound == float("inf"):
            return None, total_nodes_created, []
        current_bound = next_bound

def f_bounded_dfs(current_path, path_cost_so_far, current_bound, goal_set, sorted_graph, node_positions, total_nodes_created, best, debug=False):
    current_node = current_path[-1]
    estimated_remaining_cost = heuristic(current_node, goal_set, node_positions)
    estimated_total_cost = path_cost_so_far + estimated_remaining_cost
    # might delete
    if debug:
        print(
            f"Current={current_node}, "
            f"g={path_cost_so_far:.2f}, "
            f"h={estimated_remaining_cost:.2f}, "
            f"f={estimated_total_cost:.2f}, "
            f"bound={current_bound:.2f}, "
            f"path={current_path}"
        )
    if estimated_total_cost > current_bound:
        return False, estimated_total_cost, total_nodes_created
    if current_node in goal_set:
        candidate_path = current_path.copy()
        if best[0] is None:
            best[0] = (path_cost_so_far, candidate_path)
        else:
            best_cost, best_path = best[0]
            candidate_goal = candidate_path[-1]
            best_goal = best_path[-1]
            if (path_cost_so_far < best_cost or (path_cost_so_far == best_cost and candidate_goal < best_goal)):
                best[0] = (path_cost_so_far, candidate_path) # If cost and goal are both equal, do nothing.
        return False, float("inf"), total_nodes_created
    smallest_exceeded_bound = float("inf")
    for neighbor_node, edge_cost in sorted_graph.get(current_node, []):
        if neighbor_node in current_path:
            continue
        current_path.append(neighbor_node)
        total_nodes_created += 1
        _, returned_bound, total_nodes_created = f_bounded_dfs(current_path, path_cost_so_far + edge_cost, current_bound, goal_set, sorted_graph, node_positions, total_nodes_created, best, debug)
        if returned_bound is not None:
            smallest_exceeded_bound = min(smallest_exceeded_bound, returned_bound)
        current_path.pop()
    return False, smallest_exceeded_bound, total_nodes_created