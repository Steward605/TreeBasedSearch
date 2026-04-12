from heapq import heappush, heappop
from utils import heuristic

def greedy_best_first_search(start_node, goal_nodes, graph, node_positions, debug=False):
    goal_nodes = set(goal_nodes)
    frontier = []
    explored = set()
    number_of_nodes_created = 1
    insertion_order = 0

    start_heuristic = heuristic(start_node, goal_nodes, node_positions)
    heappush(frontier, (start_heuristic, insertion_order, start_node, [start_node]))

    while frontier:
        # pop node with smallest heuristic value
        current_heuristic, _, current_node, current_path = heappop(frontier)

        if current_node in goal_nodes:
            return current_node, number_of_nodes_created, current_path

        if current_node in explored:
            continue

        explored.add(current_node)
        outgoing_edges = graph.get(current_node, [])
        neighbor_nodes = sorted(neighbor for neighbor, cost in outgoing_edges)
        # discovered nodes are sorted in ascending node ID
        # when heuristic values are equal, smaller node IDs are preferred

        # states currently waiting in frontier
        frontier_states = {node for _, _, node, _ in frontier}

        for neighbor_node in neighbor_nodes:
            if neighbor_node not in explored and neighbor_node not in frontier_states:
                insertion_order += 1
                neighbor_heuristic = heuristic(neighbor_node, goal_nodes, node_positions)
                heappush(
                    frontier,
                    (neighbor_heuristic, insertion_order, neighbor_node, current_path + [neighbor_node])
                )
                number_of_nodes_created += 1

        if debug:
            print(
                f"Current: {current_node}, "
                f"Frontier: {[node for _, _, node, _ in frontier]}, "
                f"Explored: {sorted(explored)}"
            )

    return None, number_of_nodes_created, []