import heapq

def cus1_search(nodes, edges, origin, destinations):
    # priority_queue stores (cumulative_cost, current_node, path_taken)
    # Using a list with heapq to act as a priority queue
    frontier = [(0, origin, [origin])]
    visited = {} # Stores the lowest cost to reach a node
    nodes_created = 0

    while frontier:
        (cost, current_node, path) = heapq.heappop(frontier)
        nodes_created += 1

        # Check if we reached any of the destination nodes
        if current_node in destinations:
            return current_node, nodes_created, path

        # If we've seen this node with a lower cost, skip it
        if current_node in visited and visited[current_node] <= cost:
            continue
        visited[current_node] = cost

        # Expand neighbors based on Edges provided in the graph
        # Note: Expand in ascending order of node ID if costs are equal 
        neighbors = sorted(edges.get(current_node, [])) 

        for neighbor, edge_cost in neighbors:
            new_cost = cost + edge_cost
            if neighbor not in visited or new_cost < visited[neighbor]:
                heapq.heappush(frontier, (new_cost, neighbor, path + [neighbor]))

    return None, nodes_created, []