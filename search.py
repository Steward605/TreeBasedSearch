import os
import sys

from bfs import breadth_first_search
from astar import a_star_search
from cus2 import ida_star_search
from utils import read_route_problem
from dfs import depth_first_search
from gbfs import greedy_best_first_search  
from cus1 import cus1_search

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
        "BFS": breadth_first_search,
        "DFS": depth_first_search,
        "GBFS": greedy_best_first_search,  
        "AS": a_star_search,
        "CUS2": ida_star_search,
        "CUS1": cus1_search
    }

    if method not in search_methods:
        raise ValueError(f"Unsupported method: {method}")

    return search_methods[method]

def run_search(file_path, method, search_function):
    node_positions, graph, origin_node, destination_nodes = read_route_problem(file_path)
    # print(graph)
    
    if method in ["AS", "CUS2", "GBFS"]:
        return search_function(origin_node, destination_nodes, graph, node_positions, debug=True)
    
    if method == "CUS1":
        return search_function(node_positions, graph, origin_node, destination_nodes)

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
        goal_reached, nodes_created, path = run_search(file_path, method, search_function)
        print_search_result(file_path, method, goal_reached, nodes_created, path)

    except Exception as error:
        print(error)

if __name__ == "__main__":
    main()