import math
from queue import PriorityQueue

def a_star_search(start_node, goal_nodes, graph, node_positions, debug=False):
    # Convert goal_nodes to set for faster lookup
    goal_set = set(goal_nodes)

    # Priority queue: (f_score, count, node)
    # count ensures nodes with same f_score are processed in insertion order
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start_node))

    came_from = {}
    # g_score: cost from start to current node
    g_score = {start_node: 0}
    # f_score: g_score + heuristic (estimated cost to goal)
    f_score = {start_node: heuristic(start_node, goal_set, node_positions)}

    # Track nodes currently in open_set for duplicate checking
    open_set_hash = {start_node}

    nodes_created = 1

    if debug:
        print(f"Starting A* search from {start_node} to goals: {goal_set}")
        print("-" * 50)

    while not open_set.empty():
        # Get node with lowest f_score
        current_f, _, current_node = open_set.get()
        open_set_hash.remove(current_node)

        if debug:
            print(f"Exploring node {current_node} (f={current_f:.2f}, g={g_score[current_node]:.2f})")

        # Check if reach a goal
        if current_node in goal_set:
            if debug:
                print(f"\nGoal reached: {current_node}")
            path = reconstruct_path(came_from, current_node)
            return current_node, nodes_created, path
        
        # Explore neighbors
        outgoing_edges = graph.get(current_node, [])

        for neighbor, edge_cost in outgoing_edges:
            # Calculate tentative g_score
            tentative_g_score = g_score[current_node] + edge_cost

            # If this path to neighbor is better than any previous one
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                # This is a better path, update records
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                h_score = heuristic(neighbor, goal_set, node_positions)
                f_score[neighbor] = tentative_g_score + h_score

                # Add to open set if not already there
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    nodes_created += 1

                    if debug:
                        print(f"  Added {neighbor} to open set (g={tentative_g_score:.2f}, h={h_score:.2f}, f={f_score[neighbor]:.2f})")
                else:
                    # If already in open set, we still need to add the better path
                    # PriorityQueue doesn't support updating, so we add a new entry
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    if debug:
                        print(f"  Updated {neighbor} with better path (g={tentative_g_score:.2f})")

    # No path found
    if debug:
        print("\nNo path to any goal found!")
    return None, nodes_created, []

def heuristic(node, goal_nodes, node_positions):
    """
    Calculate Euclidean distance from node to the closest goal node.
    This is admissible (never overestimates) for A*.
    """
    if node not in node_positions:
        return 0
    
    node_x, node_y = node_positions[node]
    
    # Find minimum distance to the goal
    min_distance = float('inf')
    
    for goal in goal_nodes:
        if goal in node_positions:
            goal_x, goal_y = node_positions[goal]
            distance = math.sqrt((node_x - goal_x)**2 + (node_y - goal_y)**2)
            min_distance = min(min_distance, distance)
    
    return min_distance

def reconstruct_path(came_from, current_node):
    """
    Reconstruct the path from start to current_node using the came_from dictionary.
    """
    path = [current_node]
    while current_node in came_from:
        current_node = came_from[current_node]
        path.append(current_node)
    path.reverse()
    return path