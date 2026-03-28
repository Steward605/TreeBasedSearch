import math
from utils import heuristic


def a_star_search(start_node, goal_nodes, graph, node_positions, debug=False):
    goal_set = set(goal_nodes)
    parent_map = {}  # Maps each node to its parent on the current best path
    g_cost = {start_node: 0}  # Best known path cost (g(n) value) from start node to each node
    explored = set()  # Nodes that have already been explored
    nodes_created = 1

    # Each frontier entry stores: node, g_cost, f_cost
    frontier = [{
        "node": start_node, 
        "g_cost": 0, 
        "f_cost": heuristic(start_node, goal_set, node_positions)
    }]

    if debug:
        print(f"Starting A* search from {start_node} to goals: {goal_set}")
        print("-" * 50)

    while frontier:
        frontier.sort(key=lambda entry: (entry["f_cost"], entry["node"]))
        current_entry = frontier.pop(0)
        current_node = current_entry["node"]
        current_g_cost = current_entry["g_cost"]
        current_f_cost = current_entry["f_cost"]

        if debug:
            print(f"Exploring node {current_node} (f={current_f_cost:.2f}, g={current_g_cost:.2f})")
        if current_node in goal_set:
            if debug:
                print(f"\nGoal reached: {current_node}")
            return current_node, nodes_created, reconstruct_path(parent_map, current_node)

        explored.add(current_node)

        for neighbor_node, edge_cost in sorted(graph.get(current_node, []), key=lambda item: item[0]):
            # if neighbor_node in explored:
            #     continue
            new_g_cost = current_g_cost + edge_cost
            heuristic_cost = heuristic(neighbor_node, goal_set, node_positions)
            new_f_cost = new_g_cost + heuristic_cost
            existing_frontier_entry = find_frontier_entry(frontier, neighbor_node)

            # Check if this is a new node or a cheaper path to a known node
            if new_g_cost < g_cost.get(neighbor_node, float('inf')):
                parent_map[neighbor_node] = current_node
                g_cost[neighbor_node] = new_g_cost
                if existing_frontier_entry:
                    existing_frontier_entry["g_cost"] = new_g_cost
                    existing_frontier_entry["f_cost"] = new_f_cost
                    if debug:
                        print(f"\tUpdated node {neighbor_node} " f"(g={new_g_cost:.2f}, h={heuristic_cost:.2f}, f={new_f_cost:.2f})")
                else:
                    frontier.append({"node": neighbor_node, "g_cost": new_g_cost, "f_cost": new_f_cost})
                    if neighbor_node in explored:
                        explored.remove(neighbor_node)
                        if debug:
                            print(f"\tReopened closed node {neighbor_node} " f"(g={new_g_cost:.2f}, h={heuristic_cost:.2f}, f={new_f_cost:.2f})")
                    else:
                        nodes_created += 1
                        if debug:
                            print(f"\tAdded node {neighbor_node} " f"(g={new_g_cost:.2f}, h={heuristic_cost:.2f}, f={new_f_cost:.2f})")
    if debug:
        print("\nNo path to any goal found!")
    return None, nodes_created, []

def find_frontier_entry(frontier, target_node):
    for entry in frontier:
        if entry["node"] == target_node:
            return entry
    return None

def reconstruct_path(parent_map, goal_node):
    path = [goal_node]
    while goal_node in parent_map:
        goal_node = parent_map[goal_node]
        path.append(goal_node)
    path.reverse()
    return path