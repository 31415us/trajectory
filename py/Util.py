
from planar import Polygon,Vec2
from math import sqrt

def poly_to_tuples(p1):
   return [(v.x,v.y) for v in p1]

def path_to_tuples(path):
    return [(node.pos.x,node.pos.y) for node in path]

# should work fine for our use case...
def quadratic_solver_only_positive_solution(a,b,c):

    det = (b*b) - (4*a*c)

    if det < 0:
        return None
    elif det == 0:
        res = -b/(2*a)
        if res >= 0:
            return res
        else:
            return None
    else:
        root = sqrt(det)
        sol1 = (-b + root)/2*a
        sol2 = (-b - root)/2*a
        m = max(sol1,sol2)
        # what if both solutions are positive?!
        if m < 0:
            return None
        else:
            return m


