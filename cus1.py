import heapq

def cus1_search(nodes, edges, origin, destinations):
    frontier = [(0, origin, [origin])]
    visited = {} 
    nodes_created = 0

    while frontier:
        (cost, current_node, path) = heapq.heappop(frontier)
        nodes_created += 1

        if current_node in destinations:
            return current_node, nodes_created, path

        if current_node in visited and visited[current_node] <= cost:
            continue
        visited[current_node] = cost

        neighbors = sorted(edges.get(current_node, [])) 

        for neighbor, edge_cost in neighbors:
            new_cost = cost + edge_cost
            if neighbor not in visited or new_cost < visited[neighbor]:
                heapq.heappush(frontier, (new_cost, neighbor, path + [neighbor]))

    return None, nodes_created, []