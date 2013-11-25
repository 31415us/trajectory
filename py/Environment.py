
from planar import Vec2, Polygon

import Globals

class Enemy(object):
    
    def __init__(self,trajectory,closed_trajectory,speed):
        self.trajectory = trajectory
        self.closed_trajectory = closed_trajectory
        self.speed = speed

    def after_time(self,time):
        return None

class Environment(object):

    def __init__(self,bounding,enemies,obstacles):
        self.bounding = bounding
        self.enemies = enemies
        self.obstacles = obstacles

    def after_time(self,time):
        updated_enemies = [e.after_time(time) for e in self.enemies]
        return Environment(bounding,updated_enemies,obstacles)

    def collision(self,robot_pos,time):
        return None

