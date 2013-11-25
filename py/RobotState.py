
from planar import Vec2

class RobotState(object):

    def __init__(self,pos,speed,time_stamp):
        self.pos = pos
        self.speed = speed
        self.time_stamp = time_stamp

    def __str__(self):
        return "Position: {pos}\nSpeed: {speed}\nTime: {time}\n".format(pos = self.pos, speed = self.speed, time = self.time_stamp)

    def __eq__(self,other):
        return self.pos.almost_equals(other.pos) and self.speed.almost_equals(other.speed)

