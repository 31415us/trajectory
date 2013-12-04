
from planar import Vec2, Polygon, BoundingBox, Affine

import Globals
import Util

class Enemy(object):
    
    def __init__(self,trajectory,closed_trajectory,speed,shape):
        self.traj = trajectory      # a polygon
        self.traj_closed = closed_trajectory    # true if closed trajectory (ex: circle)
        self.speed = speed
        self.shape = shape      # a polygon

    # TODO: treat non-closed trajectories
    def position(self, time):
        dist = self.speed * time
        partial_dist = 0
        cnt = 0
        # when quitting the while loop position must lie on last line segment
        while (partial_dist <= dist):
            # plus length of next line segment
            partial_dist += (self.traj[cnt+1] - self.traj[cnt]).length
            cnt += cnt
        cnt -= cnt
        v = self.traj[cnt+1] - self.traj[cnt]
        partial_dist -= v.length
        delta_dist = dist - partial_dist
        # line = v0 + s * v1
        s = delta_dist / v.length

        return  Affine.translation(self.traj[cnt] + s * v) # affine translation matrix

    def after_time(self, time):
        return self.position(time) * self.shape


class Environment(object):

    def __init__(self,border,enemies,obstacles):
        self.border = border
        self.enemies = enemies
        self.obstacles = obstacles

    # returns True if robot collides with anything
    def collision(self,robot_pos,time):
        robot_poly = Affine.translation(robot_pos) * Globals.ROBOT_POLYGON 

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

    def border_as_tuple_list(self):
        return Util.poly_to_tuples(self.border)

    def obstacles_as_tuples(self):
        return [Util.poly_to_tuples(obs) for obs in self.obstacles]

    def enemies_as_tuples(self,time):
        return [Util.poly_to_tuples(enem.after_time(time)) for enem in self.enemies]



def interval_collision(min1,max1,min2,max2):
    if min1 < max2 and min1 > min2:
        return True

    if max1 < max2 and max1 > min2:
        return True

    return False

def bounding_box_collision(p1,p2):
    b1 = p1.bounding_box
    b2 = p2.bounding_box

    cond1 = interval_collision(b1.min_point.x,b1.max_point.x,b2.min_point.x,b2.max_point.x)
    cond2 = interval_collision(b1.min_point.y,b1.max_point.y,b2.min_point.y,b2.max_point.y)
    
    return cond1 and cond2 
    


# returns True if one polygon collides with the other one
def poly_collides_poly(p1, p2):

    if not bounding_box_collision(p1,p2):
        return False

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
        if not p1.contains_point(vert):
            return False

    return True


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

