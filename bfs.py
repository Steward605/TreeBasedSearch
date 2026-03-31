from collections import deque

def breadth_first_search(start_node, goal_nodes, graph, debug=False):
    goal_nodes = set(goal_nodes)
    frontier = deque([(start_node, [start_node])])
    explored = set()
    number_of_nodes_created = 1

    while frontier:
        # pop shallowest node in the queue
        current_node, current_path = frontier.popleft()

        if current_node in goal_nodes:
            return current_node, number_of_nodes_created, current_path

        explored.add(current_node)
        outgoing_edges = graph.get(current_node, [])
        neighbor_nodes = sorted(neighbor for neighbor, cost in outgoing_edges) # discovered nodes are sorted in ascending node ID before being appended to frontier

        # states currently waiting in frontier
        frontier_states = {node for node, _ in frontier}

        for neighbor_node in neighbor_nodes:
            if neighbor_node not in explored and neighbor_node not in frontier_states:
                frontier.append((neighbor_node, current_path + [neighbor_node]))
                number_of_nodes_created += 1

        if debug:
            print(f"Current node: {current_node} | Frontier: {[node for node, _ in frontier]} | Explored: {sorted(explored)}")

    return None, number_of_nodes_created, []