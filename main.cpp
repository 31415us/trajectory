
#include <iostream>

#include "Vector2D.hpp"
#include "Shapes.hpp"
#include "Robots.hpp"
#include "AStar.hpp"

int main(void)
{
    // setup environment

    // boundary
    std::vector<Vector2D> bound;
    bound.push_back(Vector2D(0.0,0.0));
    bound.push_back(Vector2D(3.0,0.0));
    bound.push_back(Vector2D(3.0,2.0));
    bound.push_back(Vector2D(0.0,2.0));

    // setup enemies
    std::vector<Enemy> es;

    // setup obstacles
    std::vector<Polygon> obs;

    Environment environment;
    environment.boundary = bound;
    environment.enemies = es;
    environment.obstacles = obs;

    // setup robot constraints
    RobotConstraints constr;
    constr.r = 0.25;
    constr.maxV = 0.5;
    constr.maxAcc = 0.008;

    // setup initial state
    RobotState start;
    start.pos = Vector2D(0.5,1.0);
    start.speed = Vector2D(0.0,0.0);
    start.elapsedTime = 0;

    // setup desired final state
    RobotState goal;
    goal.pos = Vector2D(2.5,1.0);
    goal.speed = Vector2D(0.0,0.0);
    // don't care about time at goal at setup
    goal.elapsedTime = 0;

    // initialize heuristic
    RobotHeuristic heuristic;
    heuristic.env = environment;
    heuristic.constraints = constr;

    // intialize Neighbours functor
    RobotNeighbours neighbours;
    neighbours.env = environment;
    neighbours.constraints = constr;

    // initialize edgeLength functor
    RobotEdgeLength edgeLength;
    edgeLength.env = environment;

    // initialize hash functor
    RobotHash hash;

    // initialize comparator
    RobotCompare comparator;

    std::vector<RobotState> path = aStar<RobotState>(start,goal,heuristic,neighbours,edgeLength,hash,comparator);

    for(auto s : path)
    {
        std::cout << s << std::endl;
    }

    return 0;
}

