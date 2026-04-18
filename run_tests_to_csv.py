import os
import csv
from pathlib import Path
from dfs import depth_first_search
from bfs import breadth_first_search
from gbfs import greedy_best_first_search
from astar import a_star_search
from cus1 import bs_search
from cus2 import ida_star_search
from utils import read_route_problem
from multi_goal_all_algorithms import (
    bfs_all_goals,
    dfs_all_goals,
    gbfs_all_goals,
    a_star_all_goals,
    cus1_bidirectional_all_goals,
    cus2_ida_star_all_goals
)

# ================= FORMAT HELPERS =================
def format_single_path(path):
    if not path:
        return "No path"
    return " > ".join(str(node) for node in path)

def format_goal_paths(goal_paths):
    if not goal_paths:
        return "None"
    return " | ".join(f"Node {goal}: {' > '.join(str(node) for node in goal_path)}" for goal, goal_path in goal_paths)

def format_found_goals(goal_paths):
    if not goal_paths:
        return "None"
    return ", ".join(str(goal) for goal, _ in goal_paths)

def format_requested_goals(destination_nodes):
    if not destination_nodes:
        return "None"
    return ", ".join(str(goal) for goal in destination_nodes)

def calculate_path_cost(path, graph):
    """Calculate the total edge cost of a single path."""
    if path is None or len(path) < 2:
        return 0
    total_cost = 0
    for current_node, next_node in zip(path, path[1:]):
        for neighbor, edge_cost in graph.get(current_node, []):
            if neighbor == next_node:
                total_cost += edge_cost
                break
    return total_cost

# ================= DISCOVER TEST FILES =================
def get_test_case_files(test_cases_dir):
    test_files = []
    if os.path.exists(test_cases_dir):
        for file_name in sorted(os.listdir(test_cases_dir)):
            if file_name.endswith("-TestCase.txt"):
                test_files.append(os.path.join(test_cases_dir, file_name))
    return test_files

# ================= ORIGINAL ALGORITHMS =================
def run_original_algorithm(method, node_positions, graph, origin_node, destination_nodes):
    if method == "DFS":
        goal_reached, nodes_created, path = depth_first_search(origin_node, destination_nodes, graph, debug=False)
    elif method == "BFS":
        goal_reached, nodes_created, path = breadth_first_search(origin_node, destination_nodes, graph, debug=False)
    elif method == "GBFS":
        goal_reached, nodes_created, path = greedy_best_first_search(origin_node, destination_nodes, graph, node_positions, debug=False)
    elif method == "AS":
        goal_reached, nodes_created, path = a_star_search(origin_node, destination_nodes, graph, node_positions, debug=False)
    elif method == "CUS1":
        goal_reached, nodes_created, path = bs_search(node_positions, graph, origin_node, destination_nodes)
    elif method == "CUS2":
        goal_reached, nodes_created, path = ida_star_search(origin_node, destination_nodes, graph, node_positions, debug=False)
    else:
        raise ValueError(f"Unsupported original method: {method}")
    cost = calculate_path_cost(path, graph) if goal_reached is not None else 0
    return {
        "path_found": "Yes" if goal_reached is not None else "No",
        "cost": cost if goal_reached is not None else "-",
        "nodes_created": nodes_created,
        "selected_goal": goal_reached if goal_reached is not None else "-",
        "path": format_single_path(path),
        "raw_path": repr(path),
    }

# ================= MULTI-GOAL ALGORITHMS =================
def run_multigoal_algorithm(method, node_positions, graph, origin_node, destination_nodes):
    if method == "MBFS":
        _, nodes_created, _, _, goal_paths = bfs_all_goals(origin_node, destination_nodes, graph, debug=False)
    elif method == "MDFS":
        _, nodes_created, _, _, goal_paths = dfs_all_goals(origin_node, destination_nodes, graph, debug=False)
    elif method == "MGBFS":
        _, nodes_created, _, _, goal_paths = gbfs_all_goals(origin_node, destination_nodes, graph, node_positions, debug=False)
    elif method == "MAS":
        _, nodes_created, _, _, goal_paths = a_star_all_goals(origin_node, destination_nodes, graph, node_positions, debug=False)
    elif method == "MCUS1":
        _, nodes_created, _, _, goal_paths = cus1_bidirectional_all_goals(origin_node, destination_nodes, graph, debug=False)
    elif method == "MCUS2":
        _, nodes_created, _, _, goal_paths = cus2_ida_star_all_goals(origin_node, destination_nodes, graph, node_positions, debug=False)
    else:
        raise ValueError(f"Unsupported multi-goal method: {method}")
    found_goal_set = {goal for goal, _ in goal_paths}
    requested_goal_set = set(destination_nodes)
    return {
        "all_goals_found": "Yes" if found_goal_set == requested_goal_set else "No",
        "goals_requested": format_requested_goals(destination_nodes),
        "goals_found": format_found_goals(goal_paths),
        "nodes_created": nodes_created,
        "goal_paths": format_goal_paths(goal_paths),
        "raw_goal_paths": repr(goal_paths),
    }

# ================= MAIN =================
def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    test_cases_dir = os.path.join(script_dir, "TestCases")
    test_files = get_test_case_files(test_cases_dir)
    if not test_files:
        print("No test case files found in TestCases directory.")
        return
    print(f"Found {len(test_files)} test cases.")
    original_methods = ["DFS", "BFS", "GBFS", "AS", "CUS1", "CUS2"]
    multigoal_methods = ["MBFS", "MDFS", "MGBFS", "MAS", "MCUS1", "MCUS2"]
    original_headers = ["Test Case","Method","Path Found","Cost","Nodes Created","Selected Goal","Path","Raw Path"]
    multigoal_headers = ["Test Case","Method","All Goals Found","Goals Requested","Goals Found","Nodes Created","Goal Paths","Raw Goal Paths"]
    original_results = []
    multigoal_results = []

    for test_file in test_files:
        test_name = os.path.basename(test_file).replace("-TestCase.txt", "")
        print(f"\nRunning tests for: {test_name}")
        node_positions, graph, origin_node, destination_nodes = read_route_problem(test_file)

        # ---------- ORIGINAL ----------
        for method in original_methods:
            print(f"  Running {method}...")
            try:
                result = run_original_algorithm(method, node_positions, graph, origin_node, destination_nodes)
                original_results.append([test_name,method,result["path_found"],result["cost"],result["nodes_created"],result["selected_goal"],result["path"],result["raw_path"]])
            except Exception as error:
                print(f" Error in {method}: {error}")
                original_results.append([test_name,method,"Error","-","-","-","Error",repr(error)])

        # ---------- MULTI-GOAL ----------
        for method in multigoal_methods:
            print(f"  Running {method}...")
            try:
                result = run_multigoal_algorithm(method, node_positions, graph, origin_node, destination_nodes)
                multigoal_results.append([test_name,method,result["all_goals_found"],result["goals_requested"],result["goals_found"],result["nodes_created"],result["goal_paths"],result["raw_goal_paths"]])
            except Exception as error:
                print(f"    Error in {method}: {error}")
                multigoal_results.append([test_name,method,"Error",format_requested_goals(destination_nodes),"Error","-","Error",repr(error)])

    original_output_file = os.path.join(script_dir, "test_results_original.csv")
    multigoal_output_file = os.path.join(script_dir, "test_results_multigoal.csv")
    with open(original_output_file, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(original_headers)
        writer.writerows(original_results)
    with open(multigoal_output_file, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(multigoal_headers)
        writer.writerows(multigoal_results)
    print(f"\n[DONE] Original results saved to: {original_output_file}")
    print(f"[DONE] Multi-goal results saved to: {multigoal_output_file}")
    print(f"[DONE] Total test cases: {len(test_files)}")
    print(f"[DONE] Original rows: {len(original_results)}")
    print(f"[DONE] Multi-goal rows: {len(multigoal_results)}")

if __name__ == "__main__":
    main()