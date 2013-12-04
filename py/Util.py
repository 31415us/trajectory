
from planar import Polygon,Vec2

def poly_to_tuples(p1):
   return [(v.x,v.y) for v in p1]

def path_to_tuples(path):
    return [(node.pos.x,node.pos.y) for node in path]
