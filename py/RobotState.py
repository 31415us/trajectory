
from planar import Vec2

import Globals

tangential_acc_quantization = 2
normal_acc_quantization = 4

dv = (Globals.ROBOT_MAX_ACC/max(tangential_acc_quantization,normal_acc_quantization)) * Globals.DELTA_T
dpos = dv * Globals.DELTA_T

class RobotState(object):

    def __init__(self,pos,speed,time_stamp):
        self.pos = pos
        self.speed = speed
        self.time_stamp = time_stamp

    def __str__(self):
        return "Position: {pos}\nSpeed: {speed}\nTime: {time}\n".format(pos = self.pos, speed = self.speed, time = self.time_stamp)

    def __eq__(self,other):
        return self.quantized() == other.quantized()

    def quantized(self):
        x = int(self.pos.x / dpos)
        y = int(self.pos.y / dpos)
        vx = int(self.speed.x / dv)
        vy = int(self.speed.y / dv)
        return (x,y,vx,vy)

    def heuristic(self,other):
        return (other.pos - self.pos).lenght() / Globals.ROBOT_MAX_V

    def edgeLength(self,other):
        return Globals.DELTA_T

    def neighbours(self,env):
        res = []
        newPos = self.pos + (self.speed * Globals.DELTA_T)
        newTimeStamp = self.time_stamp + Globals.DELTA_T

        if(env.collide(newPos,newTimeStamp)):
            return res

        tangential = self.speed.normalized() * (Globals.ROBOT_MAX_ACC / tangential_acc_quantization) 
        normal = self.speed.perpendicular() * (Globals.ROBOT_MAX_ACC / normal_acc_quantization)

        speeds = []
        for i in range(-tangential_acc_quantization, tangential_acc_quantization + 1):
            for j in range(-normal_acc_quantization, normal_acc_quantization + 1):
                s = self.speed + (i * tangential * Globals.DELTA_T) + (j * normal * Globals.DELTA_T)
                speeds.append(s.clamped(0.0,Globals.ROBOT_MAX_V))

        res = [RobotState(newPos,speed,newTimeStamp) for speed in speeds]


        return res


