
from planar import Polygon

ROBOT_MAX_ACC = 0.08 # m/s^2
ROBOT_MAX_V = 0.5 # m/s
ROBOT_RADIUS = 0.2 # m

DELTA_T = 0.1 # s

ROBOT_POLYGON = Polygon.regular(32,radius = ROBOT_RADIUS)

unit_circle_approx = Polygon.regular(24,radius = 1)
