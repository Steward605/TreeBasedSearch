def cus1_search(node_positions, edges, origin, destinations):
    f_frontier = {origin: [origin]}
    b_frontier = {dest: [dest] for dest in destinations}
    nodes_created = len(f_frontier) + len(b_frontier)

    while f_frontier and b_frontier:
        new_f_frontier = {}
        for node in sorted(f_frontier.keys()):
            path = f_frontier[node]
            for neighbor, cost in sorted(edges.get(node, [])):
                if neighbor not in f_frontier:
                    nodes_created += 1
                    new_path = path + [neighbor]
                    if neighbor in b_frontier: 
                        return neighbor, nodes_created, new_path + b_frontier[neighbor][::-1][1:]
                    new_f_frontier[neighbor] = new_path
        f_frontier = new_f_frontier

        new_b_frontier = {}
        for node in sorted(b_frontier.keys()):
            path = b_frontier[node]
            for neighbor, cost in sorted(edges.get(node, [])):
                if neighbor not in b_frontier:
                    nodes_created += 1
                    new_path = path + [neighbor]
                    if neighbor in f_frontier: 
                        return neighbor, nodes_created, f_frontier[neighbor] + new_path[::-1][1:]
                    new_b_frontier[neighbor] = new_path
        b_frontier = new_b_frontier

    return None, nodes_created, []