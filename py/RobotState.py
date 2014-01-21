
from planar import Vec2, Polygon, Affine
from heapdict import heapdict
from math import sqrt

import Globals
from Util import quadratic_solver_only_positive_solution

from memprof import *

#tangential_acc_quantization = 1
#normal_acc_quantization = 2
#
#dv = (Globals.ROBOT_MAX_ACC/max(tangential_acc_quantization,normal_acc_quantization)) * Globals.DELTA_T
#dpos = dv * Globals.DELTA_T

dv = Globals.ROBOT_MAX_ACC * Globals.DELTA_T
dpos = dv * Globals.DELTA_T

N_NEIGHBOURS = 8
DELTA_VS = Polygon.regular(N_NEIGHBOURS, radius = dv)
BRAKE_ONLY = [v for v in DELTA_VS if v.x < 0]
NO_BRAKE = [v for v in DELTA_VS if v.x >= 0]

class RobotState(object):

    def __init__(self,pos,speed,time_stamp):
        self.pos = pos
        self.speed = speed
        self.time_stamp = time_stamp
        x = int(self.pos.x / dpos)
        y = int(self.pos.y / dpos)
        vx = int(self.speed.x / dv)
        vy = int(self.speed.y / dv)
        self.quantized = (x,y,vx,vy)

    def __str__(self):
        return "Position: ({posx},{posy})\nSpeed: ({vx},{vy})\nTime: {time}\n".format(posx = self.pos.x,posy = self.pos.y,vx = self.speed.x,vy = self.speed.y,time = self.time_stamp)

    def __eq__(self,other):
        return self.quantized() == other.quantized()

    def heuristic(self,other):
        dpos = other.pos - self.pos
        angle = self.speed.normalized().dot(dpos.normalized())
        return (2 - angle) * best_straight_line_time(self.speed.length,other.speed.length,(self.pos - other.pos).length)
        #real_value = max((other.pos - self.pos).length / Globals.ROBOT_MAX_V, (other.speed - self.speed).length / Globals.ROBOT_MAX_ACC)
        #return real_value
        #return int(1*(real_value/Globals.DELTA_T))

    def edgeLength(self,other):
        return Globals.DELTA_T
        #return 1

    def braking_dist(self):
        s = self.speed.length
        return (0.5 * ((s*s) / Globals.ROBOT_MAX_ACC))

    def neighbours(self,env):
        
        newPos = self.pos + (self.speed * Globals.DELTA_T)
        newTimeStamp = self.time_stamp + Globals.DELTA_T

        if(env.collision(newPos,newTimeStamp)):
            return []

        delta_vs = Affine.rotation(self.speed.angle) * DELTA_VS

        speeds = [(self.speed + v).clamped(0.0,Globals.ROBOT_MAX_V) for v in delta_vs]
        speeds.append(self.speed)

        return [RobotState(newPos,speed,newTimeStamp) for speed in speeds]

    #def neighbours(self,env):
    #    res = []
    #    newPos = self.pos + (self.speed * Globals.DELTA_T)
    #    newTimeStamp = self.time_stamp + Globals.DELTA_T

    #    if(env.collision(newPos,newTimeStamp)):
    #        return res

    #    tangential = self.speed.normalized() * (Globals.ROBOT_MAX_ACC / tangential_acc_quantization) * Globals.DELTA_T
    #    normal = self.speed.normalized().perpendicular() * (Globals.ROBOT_MAX_ACC / normal_acc_quantization) * Globals.DELTA_T

    #    speeds = []
    #    for i in range(-tangential_acc_quantization, tangential_acc_quantization + 1):
    #        for j in range(-normal_acc_quantization, normal_acc_quantization + 1):
    #            delta_v = (i * tangential) + (j * normal)
    #            s = self.speed + delta_v.clamped(0.0,Globals.ROBOT_MAX_ACC*Globals.DELTA_T)
    #            speeds.append(s.clamped(0.0,Globals.ROBOT_MAX_V))

    #    res = [RobotState(newPos,speed,newTimeStamp) for speed in speeds]


    #    return res

# all scalar inputs
def best_straight_line_time(v1,v2,d):
    result = 0.0
    remaining_dist = d
    max_v = max(v1,v2)
    min_v = min(v1,v2)
    delta_v = max_v - min_v

    #time used and distance travelled during speed correction
    speed_correction_time = delta_v / Globals.ROBOT_MAX_ACC 
    speed_correction_distance = (min_v + (delta_v / 2)) * speed_correction_time
    if speed_correction_distance > remaining_dist:
        # though luck no solution
        return float("inf")

    remaining_dist -= speed_correction_distance
    result += speed_correction_time

    # time and distance travelled during acceleration and deceleration to robot_max_v
    delta_to_max = Globals.ROBOT_MAX_V - max_v
    acc_time = delta_to_max / Globals.ROBOT_MAX_ACC
    dist_acc_decc = ((2 * max_v) + delta_to_max) * acc_time
    if dist_acc_decc > remaining_dist:
        result += 2 * quadratic_solver_only_positive_solution(Globals.ROBOT_MAX_ACC,max_v, -(remaining_dist/2))
        return result

    remaining_dist -= dist_acc_decc
    result += 2 * acc_time

    # time to travel remaining distance

    result += remaining_dist / Globals.ROBOT_MAX_V

    return result

#@memprof(plot = True)
def aStar(start,goal,env):
    visited = set()
    parents = {}
    queue = heapdict()
    g_score = {}
    f_score = {}
    unquantized_state = {}

    quant_start = start.quantized
    unquantized_state[quant_start] = start

    g_score[quant_start] = 0
    f_score[quant_start] = g_score[quant_start] + start.heuristic(goal)

    queue[quant_start] = f_score[quant_start]

    while queue:

        current = queue.popitem()[0]

        real_current = unquantized_state[current]
        visited.add(current)


        if current == goal.quantized:
            res = []
            curr = current

            while parents.get(curr):
                res.append(curr)
                curr = parents[curr]

            res.reverse()
            return [unquantized_state[i] for i in res]

        for neigh in real_current.neighbours(env):
            quant_neigh = neigh.quantized
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

