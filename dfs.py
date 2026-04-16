def depth_first_search(start_node, goal_nodes, graph, debug=False):
    goal_nodes = set(goal_nodes)

    # frontier is a LIFO stack, so most recently generated node is expanded first
    frontier = [(start_node, [start_node])]

    # Stores nodes that have already been expanded
    explored = set()
    number_of_nodes_created = 1

    while frontier:
        current_node, current_path = frontier.pop()

        # Stop as soon as a goal node is reached
        if current_node in goal_nodes:
            return current_node, number_of_nodes_created, current_path

        explored.add(current_node)
        outgoing_edges = graph.get(current_node, [])

        # Neighbours are processed in reverse node ID order before being pushed,
        # so that the smaller node ID is expanded first when popped from the stack
        neighbor_nodes = sorted(
            (neighbor for neighbor, cost in outgoing_edges),
            reverse=True
        )

        # Avoid inserting duplicate states that are already waiting in the frontier
        frontier_states = {node for node, _ in frontier}

        for neighbor_node in neighbor_nodes:
            if neighbor_node not in explored and neighbor_node not in frontier_states:
                frontier.append((neighbor_node, current_path + [neighbor_node]))
                number_of_nodes_created += 1

        if debug:
            print(f"Current: {current_node}, Frontier: {[n for n, _ in frontier]}")

    # Failure: no goal is reachable if the frontier becomes empty
    return None, number_of_nodes_created, []

# ---------------- IMPROVED DFS ---------------- #
def depth_limited_search(start_node, goal_nodes, graph, depth_limit, debug=False):
    # Convert goal nodes to set for efficient checking
    goal_nodes = set(goal_nodes)

    # Frontier stores (node, path, depth)
    frontier = [(start_node, [start_node], 0)]

    # Count nodes created
    number_of_nodes_created = 1

    while frontier:
        # Pop last inserted node (DFS behaviour)
        current_node, current_path, current_depth = frontier.pop()

        # Goal test
        if current_node in goal_nodes:
            return current_node, number_of_nodes_created, current_path

        # Only expand if depth limit not exceeded
        if current_depth < depth_limit:
            outgoing_edges = graph.get(current_node, [])

            # Same ordering rule as DFS
            neighbor_nodes = sorted(
                (neighbor for neighbor, cost in outgoing_edges),
                reverse=True
            )

            # Track nodes already in frontier
            frontier_states = {node for node, _, _ in frontier}

            # Avoid cycles by checking current path
            path_states = set(current_path)

            for neighbor_node in neighbor_nodes:
                # Avoid revisiting nodes in current path (cycle prevention)
                # and avoid duplicates in frontier
                if neighbor_node not in path_states and neighbor_node not in frontier_states:
                    frontier.append(
                        (neighbor_node, current_path + [neighbor_node], current_depth + 1)
                    )
                    number_of_nodes_created += 1

        # Debug output
        if debug:
            print(
                f"Current: {current_node}, Depth: {current_depth}, "
                f"Frontier: {[n for n, _, _ in frontier]}"
            )

    # No solution found within depth limit
    return None, number_of_nodes_created, []


def iterative_deepening_search(start_node, goal_nodes, graph, debug=False):
    """
    Iterative Deepening DFS (IDDFS)
    --------------------------------
    Repeatedly performs depth-limited search with increasing depth limits.
    
    Advantages:
    - Combines DFS low memory usage with BFS completeness
    - Avoids infinite deep exploration
    """

    goal_nodes = set(goal_nodes)
    total_nodes_created = 0
    depth_limit = 0

    while True:
        # Run depth-limited search with current depth limit
        goal, nodes_created, path = depth_limited_search(
            start_node, goal_nodes, graph, depth_limit, debug=debug
        )

        total_nodes_created += nodes_created

        if debug:
            print(f"Depth limit {depth_limit} completed.")

        # If goal found → return result
        if goal is not None:
            return goal, total_nodes_created, path

        # Stop if no deeper nodes exist
        if not _has_node_beyond_depth(start_node, graph, depth_limit):
            return None, total_nodes_created, []

        # Increase depth limit and repeat
        depth_limit += 1


def _has_node_beyond_depth(start_node, graph, depth_limit):
    """
    Helper function:
    Checks if there are nodes beyond current depth limit.
    Prevents infinite looping in IDDFS.
    """
    frontier = [(start_node, 0)]
    visited = set()

    while frontier:
        current_node, current_depth = frontier.pop()

        if (current_node, current_depth) in visited:
            continue
        visited.add((current_node, current_depth))

        # If deeper node exists → continue IDDFS
        if current_depth > depth_limit:
            return True

        outgoing_edges = graph.get(current_node, [])
        neighbor_nodes = sorted((neighbor for neighbor, cost in outgoing_edges), reverse=True)

        for neighbor_node in neighbor_nodes:
            frontier.append((neighbor_node, current_depth + 1))

    return False