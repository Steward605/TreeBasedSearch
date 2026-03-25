from collections import deque

def breadth_first_search(start_node, goal_nodes, graph, debug=True):    
    goal_nodes = set(goal_nodes)
    frontier = deque([(start_node, [start_node])])
    visited = {start_node}
    # Count the start node as the first created node.
    number_of_nodes_created = 1

    while frontier:
        # pop shallowest node in the queue.
        current_node, current_path = frontier.popleft()
        
        # trace executions
        if debug:
            print(f"  Expanding node {current_node} | path so far: {current_path}")

        # Stop immediately if this node is one of the destination nodes.
        if current_node in goal_nodes:
            return current_node, number_of_nodes_created, current_path

        # Get all outgoing edges from the current node.
        outgoing_edges = graph.get(current_node, [])

        # Extract only neighbor node numbers.
        # Sort them so smaller node numbers are expanded first when tied.
        neighbor_nodes = sorted(neighbor for neighbor, cost in outgoing_edges)

        # Add each unvisited neighbor to the end of the queue.
        for neighbor_node in neighbor_nodes:
            if neighbor_node not in visited:
                visited.add(neighbor_node)
                number_of_nodes_created += 1
                frontier.append((neighbor_node, current_path + [neighbor_node]))

    # If queue becomes empty, no destination was reachable.
    return None, number_of_nodes_created, []