import numpy as np

from .utils import Map, Node, SearchTreePQS, compute_cost, octile_distance, make_path

def astar(graph, start_i, start_j, goal_i, goal_j, heuristic_func = octile_distance, search_tree = SearchTreePQS):
    '''
    Parameters
    ----------
    graph : Graph of states
    start_i, start_j, goal_i, goal_j : int
    heuristic_func : <function>
        default=octile_distance
    search_tree : type
        data structure for search tree in A* algorithm
        
    Returns
    -------
    path: [start, node1, node2, ..., goal]
        path between start and goal, list of consequent vertices
    '''
    # start_time = time.time()
    
    ast = search_tree()
    steps = 0
    nodes_created = 0
    # CLOSED = None
    # OPEN = None

    goal_node = Node(goal_i, goal_j, h = 0)
    start_node = Node(start_i, start_j, h = heuristic_func(start_i, start_j, goal_i, goal_j))
    
    ast.add_to_open(start_node)
    while not ast.open_is_empty():
        steps += 1
        new_node = ast.get_best_node_from_open()
        
        if (new_node == goal_node):
            # return (True, new_node, steps, nodes_created, ast.OPEN, ast.CLOSED, (time.time() - start_time))
            return make_path(new_node)
        
        if ast.was_expanded(new_node):
            continue 
            
        ast.add_to_closed(new_node)  
        
        
        for neib in graph.get_neighbors(new_node.i, new_node.j):
            next_node = Node(neib[0], neib[1], 
                             g=new_node.g + compute_cost(new_node.i, new_node.j, *neib),
                             h = heuristic_func(neib[0], neib[1], goal_i, goal_j), 
                             parent=new_node)
            
            if not ast.was_expanded(next_node):   
                nodes_created += 1             
                ast.add_to_open(next_node)
    
    # return (False, None, steps, nodes_created, ast.OPEN, ast.CLOSED, (time.time() - start_time))
    return []