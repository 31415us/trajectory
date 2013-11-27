
from planar import Vec2
from heapdict import heapdict

import Globals

tangential_acc_quantization = 1
normal_acc_quantization = 2

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
        return (other.pos - self.pos).length / Globals.ROBOT_MAX_V

    def edgeLength(self,other):
        return Globals.DELTA_T

    def neighbours(self,env):
        res = []
        newPos = self.pos + (self.speed * Globals.DELTA_T)
        newTimeStamp = self.time_stamp + Globals.DELTA_T

        if(env.collide(newPos,newTimeStamp)):
            return res

        tangential = self.speed.normalized() * (Globals.ROBOT_MAX_ACC / tangential_acc_quantization) 
        normal = self.speed.normalized().perpendicular() * (Globals.ROBOT_MAX_ACC / normal_acc_quantization)

        speeds = []
        for i in range(-tangential_acc_quantization, tangential_acc_quantization + 1):
            for j in range(-normal_acc_quantization, normal_acc_quantization + 1):
                s = self.speed + (i * tangential * Globals.DELTA_T) + (j * normal * Globals.DELTA_T)
                speeds.append(s.clamped(0.0,Globals.ROBOT_MAX_V))

        res = [RobotState(newPos,speed,newTimeStamp) for speed in speeds]


        return res

def aStar(start,goal,env):
    visited = set()
    parents = {}
    queue = heapdict()
    g_score = {}
    f_score = {}
    unquantized_state = {}

    quant_start = start.quantized()
    unquantized_state[quant_start] = start

    g_score[quant_start] = 0
    f_score[quant_start] = g_score[quant_start] + start.heuristic(goal)
    queue[quant_start] = f_score[quant_start]

    while queue:
        current = queue.popitem()[0] 
        real_current = unquantized_state[current]
        visited.add(current)

        #print current
        #print goal.quantized()

        if current == goal.quantized():
            res = []
            curr = current
            while parents.get(curr):
                res.append(curr)
                curr = parents[curr]

            res.reverse()
            return [unquantized_state[i] for i in res]

        for neigh in real_current.neighbours(env):
            quant_neigh = neigh.quantized()
            tentative_g_score = g_score[current] + real_current.edgeLength(neigh)
            tentative_f_score = tentative_g_score + neigh.heuristic(goal)
            if (quant_neigh in visited) and (tentative_f_score >= f_score[quant_neigh]):
                continue
            else:
                parents[quant_neigh] = current
                g_score[quant_neigh] = tentative_g_score
                f_score[quant_neigh] = tentative_f_score
                queue[quant_neigh] = f_score[quant_neigh]
                unquantized_state[quant_neigh] = neigh

    return None

