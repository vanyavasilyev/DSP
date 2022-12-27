import numpy as np
import itertools
import math

from heapq import heappop, heappush


class Node:
    '''
    Node class represents a search node

    - i, j: coordinates of corresponding grid element
    - g: g-value of the node
    - h: h-value of the node // always 0 for Dijkstra
    - F: f-value of the node // always equal to g-value for Dijkstra
    - parent: pointer to the parent-node 

    '''
    

    def __init__(self, i, j, g = 0, h = 0, f = None, parent = None, tie_breaking_func = None, avoid=0, path=None, clow=None):
        self.i = i
        self.j = j
        self.g = g
        self.h = h
        if f is None:
            self.f = self.g + h
        else:
            self.f = f        
        self.parent = parent
        self.path = path
        self.avoid = avoid
        self.clow = clow
        
    
    def __eq__(self, other):
        '''
        Estimating where the two search nodes are the same,
        which is needed to detect dublicates in the search tree.
        '''
        return (self.i == other.i) and (self.j == other.j)
    
    def __hash__(self):
        '''
        To implement CLOSED as set of nodes we need Node to be hashable.
        '''
        ij = self.i, self.j
        return hash(ij)


    def __lt__(self, other): 
        '''
        Comparing the keys (i.e. the f-values) of two nodes,
        which is needed to sort/extract the best element from OPEN.
        '''
        if self.avoid != other.avoid:
            return self.avoid < other.avoid
        if self.f != other.f:
            return self.f < other.f
        return self.g > other.g
    
    def __str__(self):
        return f"Node at {self.i}, {self.j} " + f"(g={self.g}, h={self.h}, avoid={self.avoid})"
    
    def __repr__(self):
        return f"Node at {self.i}, {self.j} " + f"(g={self.g}, h={self.h}, avoid={self.avoid})"


class Map:

    def __init__(self):
        '''
        Default constructor
        '''

        self._width = 0
        self._height = 0
        self._cells = []
    

    def read_from_string(self, cell_str, width, height):
        '''
        Converting a string to a grid
        '''
        self._width = width
        self._height = height
        self._cells = [[0 for _ in range(width)] for _ in range(height)]
        cell_lines = cell_str.split("\n")
        i = 0
        j = 0
        for l in cell_lines:
            if len(l) != 0:
                j = 0
                for c in l:
                    if c in ['.', 'G', 'S']:
                        self._cells[i][j] = 0
                    elif c in ['@', 'O', 'T', 'W', '#']:
                        self._cells[i][j] = 1
                    else:
                        j -= 1
                        #raise Exception("Unexpected symbol " + c)
                    j += 1
                if j != width:
                    raise Exception("Size Error. Map width = ", j, ", but must be", width )
                
                i += 1

        if i != height:
            raise Exception("Size Error. Map height = ", i, ", but must be", height )
    
     
    def set_grid_cells(self, width, height, grid_cells):
        '''
        Initialization of map by list of cells.
        '''
        self._width = width
        self._height = height
        self._cells = grid_cells


    def in_bounds(self, i, j):
        '''
        Check if the cell is on a grid.
        '''
        return (0 <= j < self._width) and (0 <= i < self._height)
    

    def traversable(self, i, j):
        '''
        Check if the cell is not an obstacle.
        '''
        return not self._cells[i][j]

    def reachable(self, i, j):
        return self.in_bounds(i, j) and self.traversable(i, j)

    def reachable_from_list(self, pairs):
        for i, j in pairs:
            if not self.reachable(i, j):
                return False
        return True

    def get_size(self):
        return (self._height, self._width)

    def get_neighbors(self, i, j): 
        neighbors = []

        for d in itertools.product([-1,0,1], repeat=2):
            if d[0] == 0 and d[1] == 0:
                continue
            if self.reachable_from_list([(i+d[0], j+d[1]), (i+d[0], j), (i, j+d[1])]):
                neighbors.append((i + d[0], j + d[1]))

        return neighbors
    
    def get_rstar_successors(self, i, j, delta, K):
        res = []
        
        angs = np.random.rand(K) * 2 * np.pi
        for ang in angs:
            n_i = int(i + delta * np.cos(ang) + 0.5)
            n_j = int(j + delta * np.sin(ang) + 0.5)
            if (n_i != i) or (n_i != j):
                if self.reachable(n_i, n_j):
                    res.append((n_i, n_j))
                
        return res


class SearchTreePQS:
    def __init__(self):
        super().__init__()
        self._open = []
        self._closed = set()

    def open_is_empty(self) -> bool:
        return len(self._open) == 0

    def add_to_open(self, item: Node) -> None:
        heappush(self._open, item)

    def get_best_node_from_open(self) -> Node:
        return heappop(self._open)

    def add_to_closed(self, item):
        self._closed.add(item)

    def was_expanded(self, item) -> bool:
        return item in self._closed

    @property
    def OPEN(self):
        return self._open
    
    @property
    def CLOSED(self):
        return self._closed


sqrt_2 = np.sqrt(2)
octile_formula_coef = sqrt_2 - 2
def octile_distance(i1, j1, i2, j2):
    i_d = (i1 - i2) if (i1 > i2) else (i2 - i1)
    j_d = (j1 - j2) if (j1 > j2) else (j2 - j1)
    min_d = min(i_d, j_d)
    return i_d + j_d + octile_formula_coef * min_d


def euclid_distance(i1, j1, i2, j2):
    return math.hypot(i1-i2, j1-j2)


def compute_cost(i1, j1, i2, j2):
    '''
    Computes cost of simple moves between cells
    '''
    if abs(i1 - i2) + abs(j1 - j2) in [1, 2]:
        return math.hypot(i1-i2, j1-j2)
    else:

        raise Exception('Trying to compute the cost of non-supported move!')
