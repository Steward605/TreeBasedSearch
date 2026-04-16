import heapq
from utils import heuristic


def a_star_search_improved(start_node, goal_nodes, graph, node_positions, debug=False):
    goal_set = set(goal_nodes)
    parent_map = {}  # Maps each node to its parent on the current best path
    g_cost = {start_node: 0}  # Best known path cost (g(n) value) from start node to each node
    explored = set()  # Nodes that have already been expanded
    nodes_created = 1
    best_goal = None
    
    # Priority queue: (f_cost, node_id, node, g_cost)
    frontier = []
    start_h = heuristic(start_node, goal_set, node_positions)
    heapq.heappush(frontier, (start_h, start_node, start_node, 0))

    # Fast lookup for nodes in frontier
    frontier_map = {start_node: (start_h, 0)}

    if debug:
        print(f"Starting A* search from {start_node} to goals: {goal_set}")
        print("-" * 50)
    while frontier:
        current_f_cost, _, current_node, current_g_cost = heapq.heappop(frontier)

        # Skip outdated entries
        if current_node not in frontier_map:
            continue

        del frontier_map[current_node]

        if debug:
            print(f"Exploring node {current_node} (f={current_f_cost:.2f}, g={current_g_cost:.2f})")
        if best_goal is not None and current_g_cost > best_goal[0]:
            break
        
        # Goal test: record the cheapest goal found so far
        if current_node in goal_set:
            candidate_path = reconstruct_path(parent_map, current_node)
            candidate = (current_g_cost, current_node, candidate_path)
            if (best_goal is None or candidate[0] < best_goal[0] or (candidate[0] == best_goal[0] and candidate[1] < best_goal[1])):
                best_goal = candidate
            continue
        explored.add(current_node)
        
        # Tiebreaking among neighbours follows ascending node ID.
        for neighbor_node, edge_cost in sorted(graph.get(current_node, []), key=lambda item: item[0]):
            new_g_cost = current_g_cost + edge_cost
            if best_goal is not None and new_g_cost > best_goal[0]:
                continue

            # Check if this is a new node or a cheaper path to a known node
            if new_g_cost < g_cost.get(neighbor_node, float('inf')):
                parent_map[neighbor_node] = current_node
                g_cost[neighbor_node] = new_g_cost
                h_cost = heuristic(neighbor_node, goal_set, node_positions)
                new_f_cost = new_g_cost + h_cost

                heapq.heappush(frontier, (new_f_cost, neighbor_node, neighbor_node, new_g_cost))
                frontier_map[neighbor_node] = (new_f_cost, new_g_cost)

                if neighbor_node in explored:
                    explored.remove(neighbor_node)
                    if debug:
                        print(f"\tReopened node {neighbor_node} (g={new_g_cost:.2f}, h={h_cost:.2f}, f={new_f_cost:.2f})")
                else:
                    nodes_created += 1
                    if debug:
                        print(f"\tAdded node {neighbor_node} (g={new_g_cost:.2f}, h={h_cost:.2f}, f={new_f_cost:.2f})")

    if best_goal is not None:
        goal_cost, goal_node, goal_path = best_goal
        if debug:
            print(f"\nGoal reached: {goal_node}")
        return goal_node, nodes_created, goal_path
    if debug:
        print("\nNo path to any goal found!")
    return None, nodes_created, []

# Reconstructs the path from the start node to the goal node using parent links
def reconstruct_path(parent_map, goal_node):
    path = [goal_node]
    while goal_node in parent_map:
        goal_node = parent_map[goal_node]
        path.append(goal_node)
    path.reverse()
    return path