
from AStar import GridState, aStar
from planar import Vec2
from RobotState import RobotState

import Globals


class TestEnv(object):
    def collide(self,pos,time):
        return False

def main():

    p = Vec2(0.0,0.0)
    v = Vec2(0.01,0.0)

    s = RobotState(p,v,0.0)

    env = TestEnv()

    neighs = s.neighbours(env)

    print len(neighs)

    for n in neighs:
        print n
        print n.quantized()
        print '\n'
    



if __name__ == "__main__":
    main()

