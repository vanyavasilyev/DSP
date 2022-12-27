import numpy as np

from .utils import Node, Map, octile_distance, SearchTreePQS, compute_cost


def wastar(w: float, grid_map: Map, start_i, start_j, goal_i, goal_j, heuristic_func=octile_distance, search_tree=SearchTreePQS,
    max_steps=None):
    if max_steps is None:
        max_steps = -1
    ast= search_tree()
    steps = 0
    nodes_created = 0
    expanded = 0
    max_len = 0
    
    ast.add_to_open(Node(start_i, start_j, 0, w * heuristic_func(start_i, start_j, goal_i, goal_j)))
    while (not ast.open_is_empty()) and (max_steps != 0):
        max_len = max(max_len, len(ast.OPEN) + len(ast.CLOSED))
        steps += 1
        max_steps -=1
        node = ast.get_best_node_from_open()
        if ast.was_expanded(node):
            continue

        if node.i == goal_i and node.j == goal_j:
            return True, node, steps, nodes_created, max_len, expanded, node.g
        ast.add_to_closed(node)
        expanded += 1
        
        for i, j in grid_map.get_neighbors(node.i, node.j):
            new_node = Node(i, j, node.g + compute_cost(i, j, node.i, node.j), w * heuristic_func(i, j, goal_i, goal_j), parent=node)
            if not ast.was_expanded(new_node):
                nodes_created += 1
                ast.add_to_open(new_node)
        
    if ast.open_is_empty():
        return False, node, steps, nodes_created, max_len, expanded, np.inf
    else:
        node = ast.get_best_node_from_open()
        return False, node, steps, nodes_created, max_len, expanded, node.f