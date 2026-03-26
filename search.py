import os
import sys

from astar import *
from utils import *


def format_path(path):
    return " ".join(str(node) for node in path)


def get_command_line_arguments():
    if len(sys.argv) != 3:
        raise ValueError("Usage: python search.py <filename> <method>")

    file_path = sys.argv[1]
    method = sys.argv[2].upper()
    return file_path, method


def get_search_function(method):
    search_methods = {
        "ASTAR": a_star_search,
    }

    if method not in search_methods:
        raise ValueError(f"Unsupported method: {method}")

    return search_methods[method]


def run_search(file_path, search_function):
    node_positions, graph, origin_node, destination_nodes = read_route_problem(file_path)

    # A star search need node position but bfs dont so just need to check the name
    if search_function.__name__ == "a_star_search":
        return search_function(origin_node, destination_nodes, graph, node_positions, debug=True)
    else:
    # Explicitly pass debug=False so the output no show debug
        return search_function(origin_node, destination_nodes, graph, debug=True)


def print_search_result(file_path, method, goal_reached, nodes_created, path):
    file_name = os.path.basename(file_path)
    print(file_name, method)

    if goal_reached is None:
        print("No goal is reachable", nodes_created)
        print("No path found")
    else:
        print(goal_reached, nodes_created)
        print(format_path(path))


def main():
    try:
        file_path, method = get_command_line_arguments()
        search_function = get_search_function(method)
        goal_reached, nodes_created, path = run_search(file_path, search_function)
        print_search_result(file_path, method, goal_reached, nodes_created, path)

    except Exception as error:
        print(error)


if __name__ == "__main__":
    main()