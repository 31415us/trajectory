
from planar import Vec2, Polygon

ROBOT_MAX_ACC = 0.008 # m/s^2
ROBOT_MAX_V = 0.5 # m/s
ROBOT_RADIUS = 0.2 # m

DELTA_T = 0.1 # s

ROBOT_POLYGON = Polygon.regular(32,radius = ROBOT_RADIUS)

unit_circle_approx = Polygon.regular(24,radius = 1)

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

class RobotState(object):

    def __init__(self,pos,speed,time_stamp):
        self.pos = pos
        self.speed = speed
        self.time_stamp = time_stamp

    def __str__(self):
        return "Position: {pos}\nSpeed: {speed}\nTime: {time}\n".format(pos = self.pos, speed = self.speed, time = self.time_stamp)

    def __eq__(self,other):
        return self.pos.almost_equals(other.pos) and self.speed.almost_equals(other.speed)
