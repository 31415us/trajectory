
from planar import Vec2, Polygon

import Globals

class Enemy(object):
    
    def __init__(self,trajectory,closed_trajectory,speed,shape):
        self.trajectory = trajectory
        self.closed_trajectory = closed_trajectory
        self.speed = speed
        self.shape = shape      # a polygon

    def after_time(self,time):
        return self.shape


class Environment(object):

    def __init__(self,border,enemies,obstacles):
        self.border = border
        self.enemies = enemies
        self.obstacles = obstacles

    # returns True if robot collides with anything
    def collision(self,robot_pos,time):
        robot_poly = Affine.translation(robot_pos) * ROBOT_POLYGON 

        if (not poly_in_poly(self.border, robot_poly)):
            return True

        for obs in self.obstacles:
            if poly_collides_poly(obs,robot_poly):
                return True

        enemy_polys = [e.after_time(time) for e in self.enemies]

        for enem in enemy_polys:
            if poly_collides_poly(enem,robot_poly):
                return True

        return False


# returns True if one polygon collides with the other one
def poly_collides_poly(p1, p2):
        if poly_in_poly(p1,p2):
            return True

        if poly_in_poly(p2,p1):
            return True

        # return True if any edges intersect
        for i in range(0, len(p1)+1):
            if i != len(p1)+1:
                ni = i+1
            else:
                ni = 0
            for j in range(0, len(p2)+1):
                if j != len(p2)+1:
                    nj = j+1
                else:
                    nj = 0
                
                return linesegments_intersect(p1[i], p1[ni], p2[j], p2[nj])


# return True if any vertex of p2 is in p1
def poly_in_poly(p1, p2):
    for vert in p2:
        if p1.contains_point(vert)
            return True

    return False


# http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
def linesegments_intersect(la1, la2, lb1, lb2):
    r1 = la2 - la1      # such that la1+t*r1 for 0<=t<=1 a linesegment
    r2 = lb2 - lb1      # such that lb1+s*r2 for 0<=s<=1 a linesegment
    v = lb1 - la1

    if r1.cross(r2) == 0.0:
        return (v.cross(r1) == 0.0)
        
    t = v.cross(r1)/r1.cross(r2)
    s = v.cross(r2)/r1.cross(r2)

    return ((0 <= t) and (1 >= t) and (0 <= s) and (1 >= s))


# returns True if a rectangle collides with an other rectangle
#def rec_collides_rec(rec1, rec2):

