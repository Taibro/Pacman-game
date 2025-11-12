import pygame
from pygame.locals import *
from vector import Vector2
from constants import *
from entity import Entity

class Pacman(Entity):
    def __init__(self, node):
        Entity.__init__(self, node)
        self.name = PACMAN
        self.directions = {STOP: Vector2(), UP: Vector2(0, -1), DOWN: Vector2(0, 1), LEFT: Vector2(-1, 0), RIGHT: Vector2(1, 0)}
        self.direction = STOP
        self.speed = 100 * TILEWIDTH/16
        self.radius = 10
        self.color = YELLOW
        self.node = node
        self.setPosition()
        self.target = node
        self.collideRadius = 5
    
    def update(self, dt):
        self.position += self.directions[self.direction] * self.speed * dt
        direction = self.getValidKey()
    
        if self.overshotTarget():
            self.node = self.target
            if self.node.neighbors[PORTAL] is not None:
                self.node = self.node.neighbors[PORTAL]
                
            self.target = self.getNewTarget(direction)
            if self.target is not self.node:
                self.direction = direction
            else:
                self.target = self.getNewTarget(self.direction)
                
            if self.target is self.node:
                self.direction = STOP
            self.setPosition()
        else:
            if self.oppositeDirection(direction):
                self.reverseDirection()
    
    def getValidKey(self):
        key_pressed = pygame.key.get_pressed()
        if key_pressed[K_UP]:
            return UP
        if key_pressed[K_DOWN]:
            return DOWN
        if key_pressed[K_RIGHT]:
            return RIGHT
        if key_pressed[K_LEFT]:
            return LEFT
        return STOP
        
    def eatPellets(self, pelletList):
        for pellet in pelletList:
            d = self.position - pellet.position
            dSquared = d.magnitudeSquared()
            rSquared = (pellet.radius + self.collideRadius) ** 2
            if dSquared <= rSquared:
                return pellet
        return None
    
    