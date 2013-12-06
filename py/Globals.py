
from planar import Polygon, Vec2

ROBOT_MAX_ACC = 1.6 # m/s^2
ROBOT_MAX_V = 0.6 # m/s
ROBOT_RADIUS = 0.2 # m

PLAYGROUND_BORDER = Polygon([Vec2(0,0),Vec2(3,0),Vec2(3,2),Vec2(0,2)])

PLAYGROUND_WIDTH = 3
PLAYGROUND_HEIGHT = 2

DELTA_T = 0.1 # s

ROBOT_POLYGON = Polygon.regular(32,radius = ROBOT_RADIUS)

unit_circle_approx = Polygon.regular(24,radius = 1)
