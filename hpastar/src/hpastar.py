import numpy as np

from .utils import Map, Node
from .astar import astar

class Cluster:
    def __init__(self, i, j, TL, TR, BR, BL) -> None:
        '''
        TL, TR, BR, BL : Node
        relative_i, relative_j: 0 <= relative coordinated of cluster's centre <= cluster_factor - 1
        '''
#         self._abstract_level = AbstractLevel
        
        self.relative_i = i
        self.relative_j = j
        self.TL = TL
        self.TR = TR
        self.BR = BR
        self.BL = BL
        
    def __eq__(self, other):
        return (self.relative_i == other.relative_i) and (self.relative_j == other.relative_j)

    def __hash__(self):
        ij = self.relative_i, self.relative_j
        return hash(ij)

    def __repr__(self):
        return (f'Cluster from ({self.TL.i},{self.TL.j}) to ({self.BR.i},{self.BR.j})')
    
    @staticmethod
    def adjacent(c1, c2) -> bool:
        return abs(c1.relative_i - c2.relative_i) + abs(c1.relative_j - c2.relative_j) == 1
    
    
class Graph:      
    '''
    vertice:
    edges: dict() = {vert : (v1, ...), }
    '''
    def __init__(self):
        self.vertice = set()
        self.edges = dict()
    
    def add_vertice(self, vert):
        self.vertice.add(vert)
        
        if vert not in self.edges.keys():
            self.edge[vert] = set()
    
    def add_edge(self, v1, v2, label:str):
        if v1 not in self.vertice:
            self.add_vertice(v1)
        
        if v2 not in self.vertice:
            self.add_vertice(v2)
            
        self.edge[v1].add(v2)
        self.edge[v2].add(v1)   
        
    def get_vertice(self, vert:Node):
        if vert in self.vertice:
            return vert
        
    def get_neighbors(self, vert):
        return self.edges[vert]
    
    def remove_vertice(self, vert):
        self.vertice -= set(vert)
        
        del self.edges[vert]
        
        for k in self.edges.keys():
            self.edges[k] -= vert
        
    
class HPAstar:
    def __init__(self, mapa : Map, max_level : int = 1):
        self._map = mapa
        self.width = -1
        self.height = -1
        self.cluster_factor = 8 # Total num of clusters = cluster_factor ** 2
        self.interedge_factor = 4 # entrance's width threshold for adding 2 inter-edges over 1. DEFAULT=6
    
        self.edges = None # : set of vertices tuples {(v1, v2),}
        self.abstract_graph = None
        self.clusters = None
        
        self.widths = None
        self.heights = None
        
    def getCluster(self, i, j):
        for c in self.clusters:
            if c.relative_i == i and c.relative_j == j:
                return c
        
    def readMap(self): # 1.1
        self.width = self._map._width
        self.height = self._map._height
        
    def buildClusters(self): # 1.2.1 
#         self.clusters = set()
        self.clusters = dict()
    
        widths = [(self.width // self.cluster_factor)] * self.cluster_factor
        for i in range(self.width % self.cluster_factor):
            widths[i] += 1
        self.widths = widths
        
        heights = [(self.height // self.cluster_factor)] * self.cluster_factor
        for i in range(self.height % self.cluster_factor):
            heights[i] += 1
        self.heights = heights            
        
        for w in range(self.cluster_factor):
            for h in range(self.cluster_factor):
                cf = self.cluster_factor
                TL = Node(sum(heights[:h]), sum(widths[:w]))
                TR = Node(sum(heights[:h]), sum(widths[:w + 1]) - 1)
                BR = Node(sum(heights[:h + 1]) - 1, sum(widths[:w + 1]) - 1)
                BL = Node(sum(heights[:h + 1]) - 1, sum(widths[:w]))
                new_cluster = Cluster(h, w, TL, TR, BR, BL)
#                 self.clusters.add(new_cluster)
                self.clusters[(h, w)] = new_cluster
    
    def getEntrancesIndex(self, border):
        first = None
        second = None    
        inter_edges = []
        for i, b in enumerate(np.array([*border, 1])):
            # b = 0: empty cell, b = 1: wall
            if b:
                if second is not None:
                    width = second - first + 1
                    if width < self.interedge_factor:
                        inter_edges += [first + width // 2]
                    else:
                        inter_edges += [first, second]
                first = None
                second = None
                continue

            if first is None:
                first = i

            second = i
        return list(set(inter_edges))
    
    def buildEntrances(self, c1: Cluster, c2: Cluster): # 1.2.2 
        cells = self._map._cells #np.array
        shift = Node(c2.relative_i - c1.relative_i, c2.relative_j - c1.relative_j)
        border, c, v = None, None, None
        if shift == (0, 1):
            border = np.logical_or(cells[c1.TR.i:c1.BR.i + 1, c1.TR.j], 
                                   cells[c1.TR.i + shift.i:c1.BR.i + shift.i + 1, c1.TR.j + shift.j])
            c = c1.TR
            v = Node(1, 0)
            
        elif shift == (-1, 0):
            border = np.logical_or(cells[c1.TL.i, c1.TL.j : c1.TR.j + 1], 
                                   cells[c1.TL.i + shift.i, c1.TL.j + shift.j: c1.TR.j + shift.j + 1])
            c = c1.TL
            v = Node(0, 1)
            
        elif shift == (1, 0):
            border = np.logical_or(cells[c1.BL.i, c1.BL.j : c1.BR.j + 1], 
                                   cells[c1.BL.i + shift.i, c1.BL.j + shift.j: c1.BR.j + shift.j + 1])
            c = c1.BL
            v = Node(0, 1)
            
        elif shift == (0, -1):
            border = np.logical_or(cells[c1.TL.i:c1.BL.i + 1, c1.TL.j], 
                                   cells[c1.TL.i + shift.i:c1.BL.i + shift.i + 1, c1.TL.j + shift.j])
            c = c1.TL
            v = Node(1, 0)
            
        if border is None:
            return

        vert_index = self.getEntrancesIndex(border)

        entrances = set()
#         print('\n', border, vert_index, v, c, shift)
        for b in vert_index:
            vert = c + v * b
            if sum(shift.VALUE) == 1:
                entrances.add((vert, vert + shift))
            else:
                entrances.add((vert + shift, vert))
        
        return entrances            
        
    def abstractMaze(self): # 1.2
        self.edges = set()
        self.buildClusters()
        
        for c1 in self.clusters.values():
            for c2 in self.clusters.values():
                if Cluster.adjacent(c1, c2):
                    self.edges = self.edges.union(self.buildEntrances(c1, c2))
    
    def get_cluster_vertices(self, cluster): # 1.3.1   
        vertices = set()

        v = cluster.TL
        while(v != cluster.TR):
            vert = self.abstract_graph.get_vertice(v)
            if vert:
                vertices.add(vert)
            vert += Node(0, 1)
            
        v = cluster.TR
        while(v != cluster.BR):
            vert = self.abstract_graph.get_vertice(v)
            if vert:
                vertices.add(vert)
            vert += Node(1, 0)
        
        v = cluster.BR
        while(v != cluster.BL):
            vert = self.abstract_graph.get_vertice(v)
            if vert:
                vertices.add(vert)
            vert += Node(0, -1)
            
        v = cluster.BL
        while(v != cluster.TL):
            vert = self.abstract_graph.get_vertice(v)
            if vert:
                vertices.add(vert)
            vert += Node(-1, 0)
        
        return vertices
                
    def buildGraph(self): # 1.3        
        self.abstract_graph = Graph()
        for e in self.edges:
            v1, v2 = e[0], e[1]
            self.abstract_graph.add_vertice(v1)
            self.abstract_graph.add_vertice(v2)
            self.abstract_graph.add_edge(v1, v2, 'INTER')
        
        for cluster in self.clusters.values():
            vertices = self.get_cluster_vertices(cluster)
            for v1 in vertices:
                for v2 in vertices:
                    if v1 == v2:
                        continue
                        
                    dist = astar(cluster, v1, v2)   
                    if dist != -1:
                        self.abstract_graph.add_edge(v1, v2, 'INTRA')
                        
    def prepocessing(self): # 1
        self.readMap()
        
        self.abstractMaze()
        
        self.buildGraph()  
        
    def determine_cluster(self, node: Node):
        y, x = node.i, node.j
        i, j = 0, 0
        
        while(sum(self.heights[:i + 1]) < y):
            i += 1
        while(sum(self.widths[:j + 1]) < x):
            j += 1
    
        return self.clusters[(i, j)]
    
    def connect_to_border(self, cluster: Cluster, node: Node):  
        vertices = self.get_cluster_vertices(cluster)
        self.abstract_graph.add_vertice(node)
        for vert in vertices:
            dist = astar(cluster, node, vert)   
            if dist != -1:
                self.abstract_graph.add_edge(node, vert, 'INTRA')
        
        
    def insert_vertice(self, node: Node):      
        cluster = self.determine_cluster(node)
        self.connect_to_border(cluster, node)
        
    def search_abstract_path(self, start, goal):  
        return astar(self.abstract_graph, start, goal)
        
    def refine_segment(self, v1, v2):  
        c1 = self.determine_cluster(v1)
        c2 = self.determine_cluster(v2)
        
        if c1 != c2:
            return [v1, v2]
        
        path = astar(c1, v1, v2)  
        return path
        
    def refine_path(self, abstract_path):  
        path = []
        v2 = abstract_path[0]
        for i in range(len(abstract_path) - 1):
            v1 = v2
            v2 = abstract_path[i + 1]
            path_segment = self.refine_segment(v1, v2)   
            if i == 0:
                path = path_segment
            else:
                path += path_segment[1:]                
        
        return path
    
    def extract(self, node: Node):  
        self.abstract_path.remove_vertice(node)
    
    def search(self, start_i, start_j, goal_i, goal_j): # 2     
        start = Node(start_i, start_j)
        goal = Node(goal_i, goal_j)
        
        self.insert_vertice(start)
        self.insert_vertice(goal)
        
        abstract_path = self.search_abstract_path(start, goal)
        lowlevel_path = self.refine_path(abstract_path)
        # smooth path
        
        self.extract(start)
        self.extract(goal)
        
        return lowlevel_path