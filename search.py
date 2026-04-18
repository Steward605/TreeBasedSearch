import os
import sys

# ================= ORIGINAL ALGORITHMS =================
from bfs import breadth_first_search
from astar import a_star_search
from cus2 import ida_star_search
from dfs import depth_first_search
from gbfs import greedy_best_first_search
from cus1 import bs_search
from improveastar import a_star_search_improved

# ================= RESEARCH MULTI-GOAL =================
from multi_goal_all_algorithms import (
    bfs_all_goals,
    dfs_all_goals_optimal,
    gbfs_all_goals_optimal,
    a_star_all_goals,
    cus1_uniform_cost_all_goals,
    ida_star_all_goals
)

# ================= INPUT =================
from utils import read_route_problem


# ================= FORMAT OUTPUT =================
def format_path(path):
    return " ".join(str(node) for node in path)


# ================= CLI =================
def get_command_line_arguments():
    if len(sys.argv) != 3:
        raise ValueError("Usage: python search.py <filename> <method>")
    file_path = sys.argv[1]
    method = sys.argv[2].upper()
    return file_path, method


# ================= METHOD SELECTOR =================
def get_search_function(method):
    search_methods = {
        # ---- Original ----
        "BFS": breadth_first_search,
        "DFS": depth_first_search,
        "GBFS": greedy_best_first_search,
        "AS": a_star_search,
        "AS2": a_star_search_improved,
        "CUS1": bs_search,
        "CUS2": ida_star_search,

        # ---- Research (Multi-goal) ----
        "MBFS": bfs_all_goals,
        "MDFS": dfs_all_goals_optimal,
        "MGBFS": gbfs_all_goals_optimal,
        "MAS": a_star_all_goals,
        "MCUS1": cus1_uniform_cost_all_goals,
        "MCUS2": ida_star_all_goals,
    }

    if method not in search_methods:
        raise ValueError(f"Unsupported method: {method}")

    return search_methods[method]


# ================= RUN SEARCH =================
def run_search(file_path, method, search_function):
    node_positions, graph, origin_node, destination_nodes = read_route_problem(file_path)

    # Methods that require heuristic (node positions)
    informed_methods = {"AS", "AS2", "GBFS", "CUS2", "MGBFS", "MAS", "MCUS2"}

    # Multi-goal research methods
    multi_goal_methods = {"MBFS", "MDFS", "MGBFS", "MAS", "MCUS1", "MCUS2"}

    # ---------- MULTI-GOAL ----------
    if method in multi_goal_methods:
        if method in informed_methods:
            return search_function(origin_node, destination_nodes, graph, node_positions, debug=False)
        else:
            return search_function(origin_node, destination_nodes, graph, debug=False)

    # ---------- ORIGINAL ----------
    if method in informed_methods:
        return search_function(origin_node, destination_nodes, graph, node_positions, debug=True)

    if method == "CUS1":
        return search_function(node_positions, graph, origin_node, destination_nodes)

    return search_function(origin_node, destination_nodes, graph, debug=True)


# ================= PRINT RESULT =================
def print_search_result(file_path, method, result):
    file_name = os.path.basename(file_path)
    print(file_name, method)

    # Handle both formats
    if len(result) == 3:
        goal_reached, nodes_created, path = result
        cost = None
    elif len(result) == 4:
        goal_reached, nodes_created, path, cost = result
    else:
        raise ValueError("Unexpected return format")

    # ---------- FAILURE ----------
    if goal_reached is None:
        print("No goal is reachable", nodes_created)
        print("No path found")
        return

    # ---------- SUCCESS ----------
    if cost is None:
        print(goal_reached, nodes_created)
    else:
        print(goal_reached, nodes_created, cost)

    print(format_path(path))


# ================= MAIN =================
def main():
    try:
        file_path, method = get_command_line_arguments()
        search_function = get_search_function(method)

        result = run_search(file_path, method, search_function)

        print_search_result(file_path, method, result)

    except Exception as error:
        print(error)


if __name__ == "__main__":
    main()