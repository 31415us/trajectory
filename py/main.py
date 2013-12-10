
from planar import Vec2, Polygon, Affine
from RobotState import RobotState, aStar
from Environment import Environment, poly_collides_poly

import pygame,sys

import Globals
import Util

# pygame setup and vars
width = 600 # px
height = 400 # px
width_conversion = width / Globals.PLAYGROUND_WIDTH
height_conversion = height / Globals.PLAYGROUND_HEIGHT
screen = pygame.display.set_mode((width,height))
white = (255,255,255)
black = (0,0,0)
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)

def main():


    start_pos = Vec2(0.5,1.0)
    start_speed = Vec2(0.0,-0.1)

    start = RobotState(start_pos,start_speed,0.0)

    goal_pos = Vec2(0.5,0.5)
    goal_speed = Vec2(0.0,0.0)

    goal = RobotState(goal_pos,goal_speed,0.0)

    obs1 = Polygon([Vec2(1,1),Vec2(1.5,1),Vec2(1,1.5)])
    obs2 = Affine.translation(Vec2(1,1.25)) * Globals.ROBOT_POLYGON

    env = Environment(Globals.PLAYGROUND_BORDER,[],[obs1])

    #if poly_collides_poly(obs1,obs2):
    #    print "(="
    #else:
    #    print ")="


    path = aStar(start,goal,env)

    #print "start:\n {s}".format(s = start)
    #print "goal:\n {g}".format(g = goal)

    #for node in path:
    #    print node

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        screen.fill(black)

        pygame.draw.lines(screen,white,True,convert_to_px_coords(env.border_as_tuple_list()),1)

        for obs in env.obstacles_as_tuples():
            pygame.draw.lines(screen,red,True,convert_to_px_coords(obs),1)

        pygame.draw.lines(screen,green,False,convert_to_px_coords(Util.path_to_tuples(path)),1)

        pygame.display.update()
    
def convert_to_px_coords(tuples):
    scaled = [(int(tup[0] * width_conversion),int(tup[1] * height_conversion)) for tup in tuples]
    invert_y = [(v[0],height - v[1]) for v in scaled]
    return invert_y



if __name__ == "__main__":
    main()

