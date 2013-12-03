
from planar import Vec2, Polygon
from RobotState import RobotState, aStar
from Environment import Environment

import Globals


def main():

    start_pos = Vec2(0.5,1.0)
    start_speed = Vec2(0.01,0.0)

    start = RobotState(start_pos,start_speed,0.0)

    goal_pos = Vec2(2.5,1.0)
    goal_speed = Vec2(0.0,0.0)

    goal = RobotState(goal_pos,goal_speed,0.0)

    env = Environment(Globals.PLAYGROUND_BORDER,[],[])

    path = aStar(start,goal,env)

    print "start:\n {s}".format(s = start)
    print "goal:\n {g}".format(g = goal)

    for node in path:
        print node


    



if __name__ == "__main__":
    main()

