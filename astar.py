import math

class PathManager:
    def __init__(self, w=0, h=0, obstacles=[]):
        self.w = w
        self.h = h
        self.obstacles = obstacles
        self.path_map = self.construct_map()

    def reset_obstacles(self, obstacles=[]):
        self.obstacles = obstacles
        self.path_map = self.construct_map()

    def construct_map(self):
        path_map = PathMap(self.w, self.h)
        path_map.update_blocked_nodes(self.obstacles)
        return path_map
        
    @staticmethod
    def dist_heuristic(from_node, to_node):
        dx = int(math.fabs(from_node[0] - to_node[0]))
        dy = int(math.fabs(from_node[1] - to_node[1]))
        return dx ** 2 + dy ** 2

    @staticmethod
    def dist(from_node, to_node):
        dx = int(math.fabs(from_node[0] - to_node[0]))
        dy = int(math.fabs(from_node[1] - to_node[1]))
        return dx if dx > dy else dy

    def generate_path(self, from_node, to_node):
        path = Path()
        path.add_open_set(from_node)
        path.set_g(from_node, 0)
        if to_node in self.path_map.blocked_nodes:
            return path.construct(from_node)
        while len(path.open_set) > 0:
            context_node = path.first_open_node()
            path.remove_open_set(context_node)
            path.add_closed_set(context_node)
            for neighbor_node in self.path_map.get_neighbor_nodes(context_node, step=15):
                if path.contains_closed_set(neighbor_node):
                    continue
                if path.close_enough(neighbor_node, to_node):
                    path.set_parent(neighbor_node, context_node)
                    return path.construct(neighbor_node)
                temp_cost = path.get_g(context_node) + PathManager.dist(context_node, neighbor_node)
                if temp_cost < path.get_g(neighbor_node):
                    path.add_open_set(neighbor_node)
                    path.set_parent(neighbor_node, context_node)
                    path.set_g(neighbor_node, temp_cost)
                    path.set_f(neighbor_node, path.get_g(neighbor_node) + PathManager.dist_heuristic(neighbor_node, to_node))
        return path.construct(from_node)


class Path:
    def __init__(self):
        self.open_set = set()
        self.closed_set = set()
        self.g = {}
        self.f = {}
        self.parent = {}

        self.path = []

    def construct(self, tile=None):
        if not tile:
            return self.path
        self.path.append(tile)
        parent = self.get_parent(tile)
        while parent:
            self.path.append(parent)
            parent = self.get_parent(parent)
        self.path.reverse()
        return self

    def get_path(self):
        return self.path

    def get_open_nodes(self):
        return [node for node in self.open_set]

    def get_closed_nodes(self):
        return [node for node in self.closed_set]

    def add_open_set(self, node):
        self.open_set.add(node)

    def remove_open_set(self, node):
        self.open_set.remove(node)

    def contains_open_set(self, node):
        return node in self.open_set

    def add_closed_set(self, node):
        self.closed_set.add(node)
    
    def remove_closed_set(self, node):
        self.closed_set.remove(node)

    def contains_closed_set(self, node):
        return node in self.closed_set

    def get_g(self, node):
        if node in self.g:
            return self.g[node]
        return float("inf")

    def set_g(self, node, value):
        self.g[node] = value

    def get_f(self, node):
        if node in self.f:
            return self.f[node]
        return 0

    def set_f(self, node, value):
        self.f[node] = value
    
    def set_parent(self, child_tile, parent_tile):
        self.parent[child_tile] = parent_tile

    def get_parent(self, node):
        if node in self.parent:
            return self.parent[node]
        return None

    def first_open_node(self):
        return sorted(self.open_set, 
            key=lambda x: self.f[x] if x in self.f else float("inf"))[0] 

    def close_enough(self, from_node, to_node):
        dx = int(from_node[0] - to_node[0])
        dy = int(from_node[1] - to_node[1])
        return (dx ** 2 + dy ** 2) <= 100


class PathMap:
    def __init__(self, w, h):
        self.w = w
        self.h = h
        self.blocked_nodes = set()

    def update_blocked_nodes(self, nodes):
        self.blocked_nodes = set()
        for node in nodes:
            self.blocked_nodes.add(node)

    def get_neighbor_nodes(self, node, step=1):
        neighbor_nodes = []
        for y in range(-1, 2):
            for x in range(-1, 2):
                if x == y == 0:
                    continue
                neighbor_node = (node[0] + x * step, node[1] + y * step)
                if neighbor_node in self.blocked_nodes:
                    continue
                else:
                    neighbor_nodes.append(neighbor_node)
        return neighbor_nodes