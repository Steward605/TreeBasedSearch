import time
import statistics
from astar import a_star_search
from improveastar import a_star_search_improved
from utils import read_route_problem

def benchmark(search_fn, runs, *args):
    times = []
    for _ in range(runs):
        start = time.perf_counter()
        search_fn(*args, debug=False)
        end = time.perf_counter()
        times.append(end - start)
    return {
        "avg_ms": statistics.mean(times) * 1000,
        "median_ms": statistics.median(times) * 1000,
        "min_ms": min(times) * 1000,
        "max_ms": max(times) * 1000,
    }

if __name__ == "__main__":
    files = [
        "medium-difficulty-TestCase.txt",
        "dead-end_multi-goals_graph-TestCase.txt",
        "simple-cycle-graph-TestCase.txt",
    ]
    runs = 5000
    for filename in files:
        node_positions, graph, start_node, goal_nodes = read_route_problem("TestCases/"+filename)
        a_star_search(start_node, goal_nodes, graph, node_positions, debug=False)
        a_star_search_improved(start_node, goal_nodes, graph, node_positions, debug=False)
        original_stats = benchmark(a_star_search, runs, start_node, goal_nodes, graph, node_positions)
        improved_stats = benchmark(a_star_search_improved, runs, start_node, goal_nodes, graph, node_positions)
        print(f"\n=== {filename} ===")
        print("Original A* :", original_stats)
        print("Improved A*:", improved_stats)