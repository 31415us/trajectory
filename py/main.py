
from planar import Vec2, Polygon, Affine
from RobotState import RobotState, aStar
from Environment import Environment, poly_collides_poly

#from pycallgraph import PyCallGraph
#from pycallgraph.output import GraphvizOutput

import pygame,sys

import Globals
import Util

#pygame setup and vars
px_per_meter = 400
width = Globals.PLAYGROUND_WIDTH * px_per_meter # px
height = Globals.PLAYGROUND_HEIGHT * px_per_meter # px
width_conversion = width / Globals.PLAYGROUND_WIDTH
height_conversion = height / Globals.PLAYGROUND_HEIGHT
pygame.init()
screen = pygame.display.set_mode((width,height))
white = (255,255,255)
black = (0,0,0)
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)

def main():


    start_pos = Vec2(0.25,1.75)
    start_speed = Vec2(0.0,-0.1)

    start = RobotState(start_pos,start_speed,0.0)

    goal_pos = Vec2(2.75,1.75)
    goal_speed = Vec2(0.0,0.0)

    goal = RobotState(goal_pos,goal_speed,0.0)

    square_vertices = [Vec2(0,0),Vec2(0.5,0),Vec2(0.5,0.5),Vec2(0,0.5)]
    square = Polygon(square_vertices,True,True)

    obs1 = Affine.translation(Vec2(0.5,1.0)) * square
    obs2 = Affine.translation(Vec2(2.0,0.5)) * square

    env = Environment(Globals.PLAYGROUND_BORDER,[],[obs1,obs2])

    #if poly_collides_poly(obs1,obs2):
    #    print "(="
    #else:
    #    print ")="


    #with PyCallGraph(output = GraphvizOutput()):
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

        for node in path:
            draw_state(node)

        pygame.display.update()

def draw_state(state):
    pygame.draw.circle(screen,red,to_px_coords((state.pos.x,state.pos.y)),2,0)
    pygame.draw.circle(screen,green,to_px_coords((state.pos.x,state.pos.y)),int(Globals.ROBOT_RADIUS * px_per_meter),1)
    #pygame.draw.circle(screen,blue,to_px_coords((state.pos.x,state.pos.y)),int((Globals.ROBOT_RADIUS + state.braking_dist()) * px_per_meter),1) 

    
def to_px_coords(tup):
    scaled = (int(tup[0] * width_conversion), int(tup[1] * height_conversion))
    return (scaled[0],height - scaled[1])

def convert_to_px_coords(tuples):
    return [to_px_coords(t) for t in tuples]



if __name__ == "__main__":
    main()

