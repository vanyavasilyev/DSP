import numpy as np
from collections import defaultdict
from dataclasses import dataclass

from .utils import Node, Map, octile_distance, SearchTreePQS, euclid_distance
from .limited_wastar import wastar


@dataclass
class rstar_metrics:
    total_steps: int = 0
    total_nodes_created: int = 0
    total_expanded: int = 0
    total_max_len: int = 0


def rstar(grid_map: Map, start_i, start_j, goal_i, goal_j, heuristic_func=octile_distance, search_tree=SearchTreePQS, w: float = 1, max_steps: int = 100, delta: float = 5, k: int = 8):
    metrics = rstar_metrics()
    
    rst = search_tree()
    avoid = defaultdict(bool)
    def add_node(node: Node):
        node.avoid = 0
        if node.g > w * heuristic_func(start_i, start_j, node.i, node.j):
            node.avoid = 1
        if (node.path is None) and (avoid[(node.i, node.j)]):
            node.avoid = 1
        rst.add_to_open(node)
        
    def reevaluate_node(node: Node):
        if node.clow is not None:
            return
        found, final_node, steps, nodes_created, max_len, expanded, clow = wastar(w, grid_map, node.parent.i, node.parent.j, node.i, node.j, heuristic_func, SearchTreePQS, max_steps)
        metrics.total_steps += steps
        metrics.total_nodes_created += nodes_created
        metrics.total_expanded += expanded
        metrics.total_max_len = max(metrics.total_max_len, max_len + len(rst.OPEN) + len(rst.CLOSED))
        
        node.clow = clow
        if found:
            node.path = final_node
        node.g = node.parent.g + node.clow
        if (not found) or (node.g >  w * heuristic_func(start_i, start_j, node.i, node.j)):
            avoid[(node.i, node.j)] = True
        
        add_node(node)
        
    rst.add_to_open(Node(start_i, start_j, 0, w * heuristic_func(start_i, start_j, goal_i, goal_j)))
    while not rst.open_is_empty():
        node = rst.get_best_node_from_open()
        if rst.was_expanded(node):
            continue
            
        if node.path is None:
            if (node.i != start_i) or (node.j != start_j):
                reevaluate_node(node)
                continue
            
        if node.i == goal_i and node.j == goal_j:
            return True, node, metrics
                
        rst.add_to_closed(node)

        succ = grid_map.get_rstar_successors(node.i, node.j, delta, k)
        if euclid_distance(node.i, node.j, goal_i, goal_j) <= delta:
            succ.append((goal_i, goal_j))
        for i,j in succ:
            new_node = Node(i, j, node.g + heuristic_func(node.i, node.j, i, j), w * heuristic_func(i, j, goal_i, goal_j), parent=node)
            if not rst.was_expanded(new_node):
                add_node(new_node)
                #print(f"Added {new_node} from {node}")
            
    return False, node, metrics