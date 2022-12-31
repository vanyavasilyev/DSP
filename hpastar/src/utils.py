import itertools
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from IPython.display import Image as Img
from PIL import Image, ImageDraw
from heapq import heappop, heappush


class Map:
    def __init__(self, map_path=None, scenario_path=None, test_count=30):
        '''
        Default constructor
        '''
        self._height = 0
        self._width = 0
        self._cells = None
        self._scenarios = None
        self.map_path = map_path
        self.scenario_path = scenario_path
        self.test_count = test_count 
        
        self.read_map_from_file()
        self.read_scenarios_from_file()
        
    def read_scenarios_from_file(self, scenario_path=None, nscnenarios=0):
        '''
        The scenario file format can be read here:
        https://movingai.com/benchmarks/formats.html
        '''
        if (scenario_path or self.scenario_path) is None:
            return
        
        if nscnenarios == 0:
            nscnenarios = self.test_count
            
        scenario_path = scenario_path or self.scenario_path
        
        df = pd.read_csv(self.scenario_path, sep='\t', names=
                         ['Bucket', 'map_path', 'width', 'height', 'start_x', 'start_y', 'goal_x', 'goal_y', 'optimal_distance'], 
                         skiprows=1)
        
        self._scenarios = df.groupby('Bucket').first().drop(['map_path', 'width', 'height'], axis=1).head(nscnenarios + 0).tail(nscnenarios).values
        self.test_count = len(self._scenarios)
        
        
    def read_map_from_file(self, map_path=None):
        '''
        The function reads map from map_path if it is specified, otherwise from self.map_path.
        
        The map has the following format:
        
        type octile
        height y
        width x
        map
        MAP_GRID(y, x)
        
        where y and x are the respective height and width of the map,
        MAP_GRID - matrix of size (y, x) - consequent rows of symbols, splitted by '\n'.
        Each symbol represent a single grid cell
        '''
        if (map_path is None) and (self.map_path is None):
            return

        map_path = map_path or self.map_path
        passable = ['.', 'G', 'S']
        unpassable = ['@', 'O', 'T', 'W']

        file = open(map_path, 'r')
        file.readline()
        self._height = int(file.readline().split()[1])
        self._width = int(file.readline().split()[1])
        file.readline()
        
        self._cells = np.zeros((self._height, self._width)) # (y, x) sizes
        
        for i, line in enumerate(file):
            for j, cell in enumerate(line):
                if cell in passable:
                    self._cells[i, j] = 0
                elif cell in unpassable:
                    self._cells[i, j] = 1
                elif cell in ['\n', '\t']:
                    continue
                else:
                    raise Exception(f"Cell ({i}, {j}) has in wrong format:{cell}")

        file.close()

    
    def read_from_string(self, cell_str, width, height):
        pass
    
     
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
        i = int(i)
        j = int(j)
        return (0 <= j < self._width) and (0 <= i < self._height)
    

    def traversable(self, i, j):
        '''
        Check if the cell is not an obstacle.
        '''
        i = int(i)
        j = int(j)
        return self._cells[i, j] == 0

    def jumpable(self, i, j, jump):
        '''
        Check if the jump does not cut the corners
        '''
        i = int(i)
        j = int(j)
        return self.traversable(i + jump[0], j) and self.traversable(i, j + jump[1])

    def get_neighbors(self, i, j):
        '''
        Get a list of neighbouring cells as (i,j) tuples.
        8-connected grid
        '''   
        i = int(i)
        j = int(j)
        neighbors = []
        delta = list(itertools.product([-1, 0, 1], repeat=2)) # 8-connected
        del delta[4]

        for d in delta:
            if self.in_bounds(i + d[0], j + d[1]) and self.traversable(i + d[0], j + d[1]) and self.jumpable(i, j, d): #TODO
                neighbors.append((i + d[0], j + d[1]))

        return neighbors

    
    def get_size(self):
        return (self._height, self._width)
    
def draw(grid_map, start = None, goal = None, path = None, nodes_opened = None, nodes_expanded = None, nodes_reexpanded = None):
    '''
    Auxiliary function that visualizes the environment, the path and 
    the open/expanded/re-expanded nodes.
    
    The function assumes that nodes_opened/nodes_expanded/nodes_reexpanded
    are iterable collestions of SearchNodes
    '''
    k = 20
    height, width = grid_map.get_size()
    h_im = height * k
    w_im = width * k
    im = Image.new('RGB', (w_im, h_im), color = 'white')
    draw = ImageDraw.Draw(im)
    
    for i in range(height):
        for j in range(width):
            if(not grid_map.traversable(i, j)):
                draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=( 33, 33, 33 ))

    if nodes_opened is not None:
        for node in nodes_opened:
            draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(213, 219, 219), width=0)
    
    if nodes_expanded is not None:
        for node in nodes_expanded:
            draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(131, 145, 146), width=0)
    
    if nodes_reexpanded is not None:
        for node in nodes_reexpanded:
                draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(255, 145, 146), width=0)
    
    if path is not None:
        for step in path:
            if (step is not None):
                if (grid_map.traversable(step.i, step.j)):
                    draw.rectangle((step.j * k, step.i * k, (step.j + 1) * k - 1, (step.i + 1) * k - 1), fill=(150, 80, 100), width=0)
                else:
                    draw.rectangle((step.j * k, step.i * k, (step.j + 1) * k - 1, (step.i + 1) * k - 1), fill=(230, 126, 34), width=0)

    if (start is not None) and (grid_map.traversable(start.i, start.j)):
        draw.rectangle((start.j * k, start.i * k, (start.j + 1) * k - 1, (start.i + 1) * k - 1), fill=(40, 180, 99), width=0)
    
    if (goal is not None) and (grid_map.traversable(goal.i, goal.j)):
        draw.rectangle((goal.j * k, goal.i * k, (goal.j + 1) * k - 1, (goal.i + 1) * k - 1), fill=(231, 76, 60), width=0)


    _, ax = plt.subplots(dpi=150)
    ax.axes.xaxis.set_visible(False)
    ax.axes.yaxis.set_visible(False)
    plt.imshow(np.asarray(im))
    plt.show()

    
class Node:
    '''
    Node class represents a search node

    - i, j: coordinates of corresponding grid element
        i = y, j = x
    - g: g-value of the node
    - h: h-value of the node // always 0 for Dijkstra
    - F: f-value of the node // always equal to g-value for Dijkstra
    - parent: pointer to the parent-node 

    '''

    def __init__(self, i, j, g = 0., h = 0., f = None, parent = None):
        self.i = i
        self.j = j
        self.g = g
        self.h = h
        if f is None:
            self.f = self.g + h
        else:
            self.f = f        
        self.parent = parent
        
    
    def __eq__(self, other):
        '''
        Estimating where the two search nodes are the same,
        which is needed to detect dublicates in the search tree.
        '''
        if isinstance(other, tuple):
            return (self.i == other[0]) and (self.j == other[1])
            
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
        
        This comparator is very basic. We will code a more plausible comparator further on.
        '''
        if not(self.f == other.f):
            return self.f < other.f
        return self.h < other.h  # or compare by g-value
    
    def __add__(self, other):
        return Node(self.i + other.i, self.j + other.j)
    
    def __mul__(self, num:int):
        return Node(self.i * num, self.j * num)
    
    def __repr__(self):
        return str((self.i, self.j))
    
    @property
    def VALUE(self):
        return (self.i, self.j) 

class SearchTreePQS:
    def __init__(self):
        self._open = []           # PQ for the OPEN nodes
        self._closed = set()     # SET for the expanded nodes in CLOSED
        
    def __len__(self):
        return len(self._open) + len(self._closed)
    
    def open_is_empty(self):
        return len(self._open) == 0
    
    def add_to_open(self, item):
        heappush(self._open, item)

    def get_best_node_from_open(self):
        return heappop(self._open)

    def add_to_closed(self, item):
        self._closed.add(item)

    def was_expanded(self, item):
        return item in self._closed

    @property
    def OPEN(self):
        return self._open
    
    @property
    def CLOSED(self):
        return self._closed
    
def compute_cost(i1, j1, i2, j2):
    '''
    Computes cost of simple moves between cells
    '''
    if i1 == i2 and j1 == j2:
        return 0
    elif abs(i1 - i2) + abs(j1 - j2) == 1: #cardinal move
        return 1
    elif abs(i1 - i2) == abs(j1 - j2) == 1:
        return np.sqrt(2)
    else:
        raise Exception('Trying to compute the cost of non-supported move! ONLY cardinal moves are supported.')

def octile_distance(x1, y1, x2, y2):
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    return 1 * abs(dx - dy) + np.sqrt(2) * min(dx, dy)

def make_path(goal):
    '''
    Creates a path by tracing parent pointers from the goal node to the start node
    It also returns path's length.
    '''

    length = goal.g
    current = goal
    path = []
    while current.parent:
        path.append(current)
        current = current.parent
    path.append(current)
    return path[::-1], length
