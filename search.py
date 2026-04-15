import os
import sys

from bfs import breadth_first_search
from astar import a_star_search
from cus2 import ida_star_search
from utils import read_route_problem

# Converts the path list into the required output format
def format_path(path):
    return " ".join(str(node) for node in path)

# Reads the input file name and search method from the command line
def get_command_line_arguments():
    if len(sys.argv) != 3:
        raise ValueError("Usage: python search.py <filename> <method>")
    file_path = sys.argv[1]
    method = sys.argv[2].upper()
    return file_path, method

# Selects the search function that matches the requested method
def get_search_function(method):
    search_methods = {
        "BFS": breadth_first_search,
        "AS": a_star_search,
        "CUS2": ida_star_search
    }
    if method not in search_methods:
        raise ValueError(f"Unsupported method: {method}")
    return search_methods[method]

# Reads the problem file and runs the selected search algorithm
def run_search(file_path, method, search_function):
    node_positions, graph, origin_node, destination_nodes = read_route_problem(file_path)
    if method == "AS" or method == "CUS2":
        return search_function( origin_node, destination_nodes, graph, node_positions, debug=True)
    # Explicitly pass debug=False when the output dont want debug
    return search_function(origin_node, destination_nodes, graph, debug=True)

# Prints the result in the assignment's required output format
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
        goal_reached, nodes_created, path = run_search(file_path, method, search_function)
        print_search_result(file_path, method, goal_reached, nodes_created, path)
    except Exception as error:
        print(error)

if __name__ == "__main__":
    main()