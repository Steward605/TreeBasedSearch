import math
import heapq


def a_star_search(start_node, goal_nodes, graph, node_positions, debug=False):
    goal_set = set(goal_nodes)
    came_from = {}
    g_score = {start_node: 0}
    insertion_order = 0
    frontier = [(heuristic(start_node, goal_set, node_positions), start_node, insertion_order, 0)]
    nodes_created = 1

    if debug:
        print(f"Starting A* search from {start_node} to goals: {goal_set}")
        print("-" * 50)

    while frontier:
        current_f, current_node, _, current_g = heapq.heappop(frontier)

        if current_g > g_score.get(current_node, float("inf")):
            continue

        if debug:
            print(f"Exploring node {current_node} (f={current_f:.2f}, g={current_g:.2f})")

        if current_node in goal_set:
            if debug:
                print(f"\nGoal reached: {current_node}")
            return current_node, nodes_created, reconstruct_path(came_from, current_node)

        for neighbor, edge_cost in sorted(graph.get(current_node, []), key=lambda x: x[0]):
            tentative_g = current_g + edge_cost
            is_new_node = neighbor not in g_score

            if is_new_node or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g
                h_score = heuristic(neighbor, goal_set, node_positions)
                insertion_order += 1
                heapq.heappush(frontier, (tentative_g + h_score, neighbor, insertion_order, tentative_g))

                if is_new_node:
                    nodes_created += 1

                if debug:
                    action = "Added" if is_new_node else "Updated"
                    print(f"  {action} {neighbor} (g={tentative_g:.2f}, h={h_score:.2f}, f={tentative_g + h_score:.2f})")

    if debug:
        print("\nNo path to any goal found!")
    return None, nodes_created, []


def heuristic(node, goal_nodes, node_positions):
    if node not in node_positions:
        return 0
    nx, ny = node_positions[node]
    return min(
        (math.sqrt((nx - node_positions[g][0])**2 + (ny - node_positions[g][1])**2)
         for g in goal_nodes if g in node_positions),
        default=0
    )


def reconstruct_path(came_from, current_node):
    path = [current_node]
    while current_node in came_from:
        current_node = came_from[current_node]
        path.append(current_node)
    path.reverse()
    return path