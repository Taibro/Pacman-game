import pygame
from pygame.locals import *
from vector import Vector2
from constants import *
from entity import Entity
from modes import ModeController
from sprites import GhostSprites
import heapq
from collections import deque
import random

# --- CÁC THUẬT TOÁN TÌM ĐƯỜNG ---

def heuristic(node_a, node_b):
    # Sử dụng khoảng cách Manhattan cho lưới vuông
    (x1, y1) = node_a.position.asTuple()
    (x2, y2) = node_b.position.asTuple()
    return abs(x1 - x2) + abs(y1 - y2)

def get_direction_from_path(start_node, next_node):
    # Xác định hướng đi dựa trên node hiện tại và node tiếp theo trong đường đi
    diff = next_node.position - start_node.position
    if diff.x > 0: return RIGHT
    if diff.x < 0: return LEFT
    if diff.y > 0: return DOWN
    if diff.y < 0: return UP
    return STOP

def algo_bfs(start_node, target_node, grid_access_check_entity=None):
    queue = deque([(start_node, [start_node])])
    visited = set([start_node])

    while queue:
        (current, path) = queue.popleft()
        print('bfs ',current)
        if current == target_node:
            return path
        
        for direction, neighbor in current.neighbors.items():
            # Bỏ qua PORTAL để tránh lỗi KeyError và lỗi di chuyển xuyên map
            if direction == PORTAL:
                continue

            # Kiểm tra xem hướng đó có đi được không
            can_move = True
            if grid_access_check_entity:
                if direction in current.access and grid_access_check_entity.name in current.access[direction]:
                     pass
                else:
                    can_move = False
            
            if neighbor and neighbor not in visited and can_move:
                visited.add(neighbor)
                queue.append((neighbor, path + [neighbor]))
    return []

def algo_dfs(start_node, target_node, grid_access_check_entity=None):
    stack = [(start_node, [start_node])]
    visited = set([start_node])

    while stack:
        (current, path) = stack.pop()
        print('dfs ', current)
        if current == target_node:
            return path
        
        neighbors_list = []
        for direction, neighbor in current.neighbors.items():
             # Bỏ qua PORTAL
             if direction == PORTAL:
                 continue

             can_move = True
             if grid_access_check_entity and neighbor:
                # Kiểm tra access an toàn hơn
                if direction in current.access:
                    if grid_access_check_entity.name not in current.access[direction]:
                        can_move = False
                else:
                    can_move = False # Nếu không có key trong access thì coi như không đi được

             if neighbor and can_move:
                 neighbors_list.append(neighbor)
        
        for neighbor in neighbors_list:
            if neighbor not in visited:
                visited.add(neighbor)
                stack.append((neighbor, path + [neighbor]))
    return []

def algo_astar(start_node, target_node, grid_access_check_entity=None,randomness=0):
    frontier = []
    heapq.heappush(frontier, (0, id(start_node), start_node))
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}

    while frontier:
        _, _, current = heapq.heappop(frontier)
        print('a* ',current)
        if current == target_node:
            break

        for direction, neighbor in current.neighbors.items():
            # Bỏ qua PORTAL
            if direction == PORTAL:
                continue

            can_move = True
            if grid_access_check_entity and neighbor:
                if direction in current.access:
                    if grid_access_check_entity.name not in current.access[direction]:
                        can_move = False
                else:
                    can_move = False

            if neighbor and can_move:
                added_cost = 1 + random.uniform(0, randomness)
                new_cost = cost_so_far[current] + added_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, target_node)
                    heapq.heappush(frontier, (priority, id(neighbor), neighbor))
                    came_from[neighbor] = current
    
    if target_node not in came_from:
        return []
    
    path = []
    curr = target_node
    while curr != start_node:
        path.append(curr)
        curr = came_from[curr]
    path.reverse()
    return [start_node] + path

# --- GHOST CLASSES ---

class Ghost(Entity):
    def __init__(self, node, pacman=None, blinky=None):
        Entity.__init__(self,node)
        self.name = GHOST
        self.points = 200
        self.goal = Vector2()
        self.pacman = pacman
        self.mode = ModeController(self)
        self.blinky = blinky
        self.homeNode = node
        self.use_algorithm = False
        self.algorithm_type = None 
        self.astar_randomness = 0
        
    def update(self, dt):
        self.sprites.update(dt)
        self.mode.update(dt)
        if self.mode.current is SCATTER:
            self.scatter()
        elif self.mode.current is CHASE:
            self.chase()
        Entity.update(self, dt)
        
    def scatter(self):
        self.goal = Vector2()
        
    def chase(self):
        self.goal = self.pacman.position
        
    def startFreight(self):
        self.mode.setFreightMode()
        if self.mode.current == FREIGHT:
            self.setSpeed(50)
            self.directionMethod = self.randomDirection
            
    def normalMode(self):
        self.setSpeed(100)
        if self.use_algorithm:
            self.directionMethod = self.algoDirection
        else:
            self.directionMethod = self.goalDirection
        self.homeNode.denyAccess(DOWN, self)
        
    def spawn(self):
        self.goal = self.spawnNode.position
        
    def setSpawnNode(self, node):
        self.spawnNode = node
        
    def startSpawn(self):
        self.mode.setSpawnMode()
        if self.mode.current == SPAWN:
            self.setSpeed(150)
            self.directionMethod = self.goalDirection
            self.spawn()
            
    def reset(self):
        Entity.reset(self)
        self.points = 200
        self.use_algorithm = False
        self.directionMethod = self.goalDirection

    def algoDirection(self, directions):
        if not self.pacman or not self.node:
            return self.randomDirection(directions)

        target_node = self.pacman.node
        path = []
        
        if self.algorithm_type == 'A*':
            path = algo_astar(self.node, target_node, self,self.astar_randomness)
        elif self.algorithm_type == 'BFS':
            path = algo_bfs(self.node, target_node, self)
        elif self.algorithm_type == 'DFS':
            path = algo_dfs(self.node, target_node, self)
            
        if len(path) > 1:
            next_node = path[1]
            new_dir = get_direction_from_path(self.node, next_node)
            if new_dir in directions:
                return new_dir
            
        return self.goalDirection(directions)

class Blinky(Ghost):
    def __init__(self, node, pacman=None, blinky=None):
        super().__init__(node, pacman, blinky)
        self.name = BLINKY
        self.color = RED
        self.sprites = GhostSprites(self)
        self.algorithm_type = 'A*'
        self.astar_randomness = 0.5
        
class Pinky(Ghost):
    def __init__(self, node, pacman=None, blinky=None):
        super().__init__(node, pacman, blinky)
        self.name = PINKY
        self.color = PINK
        self.sprites = GhostSprites(self)
        self.algorithm_type = 'DFS'
        
    def scatter(self):
        self.goal = Vector2(TILEWIDTH * NCOLS, 0)
        
    def chase(self):
        self.goal = self.pacman.position + self.pacman.directions[self.pacman.direction] * TILEWIDTH * 4
        
class Inky(Ghost):
    def __init__(self, node, pacman=None, blinky=None):
        super().__init__(node, pacman, blinky)
        self.name = INKY
        self.color = TEAL
        self.sprites = GhostSprites(self)
        self.algorithm_type = 'A*'
        self.astar_randomness = 5.0
        
    def scatter(self):
        self.goal = Vector2(TILEWIDTH * NCOLS, TILEHEIGHT * NROWS)
        
    def chase(self):
        vec1 = self.pacman.position + self.pacman.directions[self.pacman.direction] * TILEWIDTH * 2
        vec2 = (vec1 - self.blinky.position) * 2
        self.goal = self.blinky.position + vec2
        
class Clyde(Ghost):
    def __init__(self, node, pacman=None, blinky=None):
        super().__init__(node, pacman, blinky)
        self.name = CLYDE
        self.color = ORANGE
        self.sprites = GhostSprites(self)
        self.algorithm_type = 'BFS'
        
    def scatter(self):
        self.goal = Vector2(0, TILEHEIGHT * NROWS)
        
    def chase(self):
        d = self.pacman.position - self.position
        ds = d.magnitudeSquared()
        if ds <= (TILEWIDTH * 8) ** 2:
            self.scatter()
        else:
            self.goal = self.pacman.position + self.pacman.directions[self.pacman.direction] * TILEWIDTH * 4
            
class GhostGroup(object):
    def __init__(self, node, pacman):
        self.blinky = Blinky(node, pacman)
        self.pinky = Pinky(node, pacman)
        self.inky = Inky(node, pacman, self.blinky)
        self.clyde = Clyde(node, pacman)
        self.ghosts = [self.blinky, self.pinky, self.inky, self.clyde]
        
    def __iter__(self):
        return iter(self.ghosts)
    
    def update(self, dt):
        for ghost in self:
            ghost.update(dt)
            
    def startFreight(self):
        for ghost in self:
            ghost.startFreight()
        self.resetPoints()
        
    def setSpawnNode(self, node):
        for ghost in self:
            ghost.setSpawnNode(node)
            
    def updatePoints(self):
        for ghost in self:
            ghost.points *= 2
            
    def resetPoints(self):
        for ghost in self:
            ghost.points = 200
            
    def hide(self):
        for ghost in self:
            ghost.visible = False
            
    def show(self):
        for ghost in self:
            ghost.visible = True
            
    def render(self, screen):
        for ghost in self:
            ghost.render(screen)

    def reset(self):
        for ghost in self:
            ghost.reset()
            
    def trigger_ai_chase(self, home_node, algo_type=None): 
        for ghost in self:
            ghost.position = home_node.position.copy()
            ghost.node = home_node
            ghost.target = home_node
            ghost.direction = STOP
            ghost.visible = True
            ghost.use_algorithm = True
          
            if algo_type is not None:
                ghost.algorithm_type = algo_type
            # ----------------------
            
            ghost.directionMethod = ghost.algoDirection
            ghost.mode.current = CHASE
            ghost.setSpeed(100)