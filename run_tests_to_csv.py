import os
import sys
import csv
from pathlib import Path

from dfs import depth_first_search
from bfs import breadth_first_search
from gbfs import greedy_best_first_search
from astar import a_star_search
from cus1 import bs_search
from cus2 import ida_star_search
from utils import read_route_problem
from improveastar import a_star_search_improved
from multi_goal_all_algorithms import (
    bfs_all_goals,
    dfs_all_goals_optimal,
    gbfs_all_goals_optimal,
    a_star_all_goals,
    cus1_uniform_cost_all_goals,
    ida_star_all_goals
)

def calculate_path_cost(path, graph):
    """Calculate the total cost of a path."""
    if path is None or len(path) < 2:
        return 0
    
    total_cost = 0
    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]
        
        # Find the edge cost
        if current_node in graph:
            for neighbor, cost in graph[current_node]:
                if neighbor == next_node:
                    total_cost += cost
                    break
    
    return total_cost

def run_search_algorithm(file_path, method, node_positions, graph, origin_node, destination_nodes):
    """Run a search algorithm and return results."""
    try:
        if method == "DFS":
            goal_reached, nodes_created, path = depth_first_search(origin_node, destination_nodes, graph, debug=True)
        elif method == "BFS":
            goal_reached, nodes_created, path = breadth_first_search(origin_node, destination_nodes, graph, debug=True)
        elif method == "GBFS":
            goal_reached, nodes_created, path = greedy_best_first_search(origin_node, destination_nodes, graph, node_positions, debug=True)
        elif method == "A*":
            goal_reached, nodes_created, path = a_star_search(origin_node, destination_nodes, graph, node_positions, debug=True)
        elif method == "CUS1":
            nodes = list(graph.keys())
            goal_reached, nodes_created, path = bs_search(nodes, graph, origin_node, destination_nodes)
        elif method == "CUS2":
            goal_reached, nodes_created, path = ida_star_search(origin_node, destination_nodes, graph, node_positions, debug=True)
        elif method == "Improved A*":
            goal_reached, nodes_created, path = a_star_search_improved(origin_node, destination_nodes, graph, node_positions, debug=True)
        elif method == "Multi-Goal BFS":
            goal_reached, nodes_created, path, cost = bfs_all_goals(origin_node, destination_nodes, graph, debug=True)
            return goal_reached, nodes_created, cost, " ".join(str(node) for node in path) if path else "No path", path
        elif method == "Multi-Goal DFS":
            goal_reached, nodes_created, path, cost = dfs_all_goals_optimal(origin_node, destination_nodes, graph, debug=True)
            return goal_reached, nodes_created, cost, " ".join(str(node) for node in path) if path else "No path", path
        elif method == "Multi-Goal GBFS":
            goal_reached, nodes_created, path, cost = gbfs_all_goals_optimal(origin_node, destination_nodes, graph, node_positions, debug=True)
            return goal_reached, nodes_created, cost, " ".join(str(node) for node in path) if path else "No path", path
        elif method == "Multi-Goal A*":
            goal_reached, nodes_created, path, cost = a_star_all_goals(origin_node, destination_nodes, graph, node_positions, debug=True)
            return goal_reached, nodes_created, cost, " ".join(str(node) for node in path) if path else "No path", path
        elif method == "Multi-Goal CUS1":
            goal_reached, nodes_created, path, cost = cus1_uniform_cost_all_goals(origin_node, destination_nodes, graph, debug=True)
            return goal_reached, nodes_created, cost, " ".join(str(node) for node in path) if path else "No path", path
        elif method == "Multi-Goal CUS2":
            goal_reached, nodes_created, path, cost = ida_star_all_goals(origin_node, destination_nodes, graph, node_positions, debug=True)
            return goal_reached, nodes_created, cost, " ".join(str(node) for node in path) if path else "No path", path
        else:
            return None, None, None, None, None
        
        # Calculate cost
        cost = calculate_path_cost(path, graph) if path else 0
        
        # Format path
        formatted_path = " ".join(str(node) for node in path) if path else "No path"
        
        return goal_reached, nodes_created, cost, formatted_path, path
    
    except Exception as e:
        print(f"Error running {method} on {file_path}: {e}")
        import traceback
        traceback.print_exc()
        return None, None, None, None, None

def get_test_case_files(test_cases_dir):
    """Get all .txt test case files."""
    test_files = []
    if os.path.exists(test_cases_dir):
        for file in sorted(os.listdir(test_cases_dir)):
            if file.endswith("-TestCase.txt"):
                test_files.append(os.path.join(test_cases_dir, file))
    return test_files

def main():
    # Get the directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    test_cases_dir = os.path.join(script_dir, "TestCases")
    
    # Get all test case files
    test_files = get_test_case_files(test_cases_dir)
    
    if not test_files:
        print("No test case files found in TestCases directory.")
        return
    
    print(f"Found {len(test_files)} test cases.")
    
    # Define headers and algorithms
    headers = ["Test Case", "Method", "Path Found", "Cost", "Nodes Created", "Selected Goal", "Path", "Raw Output"]
    algorithms = ["DFS", "BFS", "GBFS", "A*", "CUS1", "CUS2", "Improved A*", "Multi-Goal BFS", "Multi-Goal DFS", "Multi-Goal GBFS", "Multi-Goal A*", "Multi-Goal CUS1", "Multi-Goal CUS2"]
    
    # Run tests
    results = []
    for test_file in test_files:
        test_name = os.path.basename(test_file).replace("-TestCase.txt", "")
        print(f"\nRunning tests for: {test_name}")
        
        # Load test case data once
        node_positions, graph, origin_node, destination_nodes = read_route_problem(test_file)
        
        # Run all algorithms
        for method in algorithms:
            print(f"  Running {method}...")
            goal, nodes_created, cost, path_formatted, raw_path = run_search_algorithm(test_file, method, node_positions, graph, origin_node, destination_nodes)
            path_found = "Yes" if goal is not None else "No"
            results.append([test_name, method, path_found, cost if goal else "-", nodes_created, goal if goal else "-", path_formatted, str(raw_path)])
    
    # Save to CSV
    output_file = os.path.join(script_dir, "test_results.csv")
    with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)
        writer.writerows(results)
    
    print(f"\n[DONE] Results saved to: {output_file}")
    print(f"[DONE] Total test cases: {len(test_files)}")
    print(f"[DONE] Total results: {len(results)}")

if __name__ == "__main__":
    main()