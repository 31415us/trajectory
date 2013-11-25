
#pragma once

#include <vector>

#include "Shapes.hpp"

struct RobotConstraints
{
    double maxAcc;
    double maxV;
    double r;
};

struct Enemy
{
    Circle circ;
    std::vector<Vector2D> trajectory;
    bool closedTrajectory;
    Enemy(){}
    Enemy getAfterTime(unsigned int t);
};

struct Environment
{
    Polygon boundary;
    std::vector<Polygon> obstacles;
    std::vector<Enemy> enemies;
    Environment(){}
    Environment getAfterTime(unsigned int t);
    bool isSound(const Circle & circ) const;
};

struct RobotState
{
    Vector2D pos;
    Vector2D speed;
    unsigned int elapsedTime;
};

std::ostream & operator<<(std::ostream & os, const RobotState & v);

struct RobotHeuristic
{
    Environment env;
    RobotConstraints constraints;
    double operator()(RobotState s1, RobotState s2);
};

struct RobotNeighbours
{
    Environment env;
    RobotConstraints constraints;
    std::vector<RobotState> operator()(RobotState s);
};

struct RobotEdgeLength
{
    Environment env;
    double operator()(RobotState s1, RobotState s2);
};

struct RobotHash
{
    size_t operator()(RobotState s);
};

struct RobotCompare
{
    static const double EPSILON;
    bool operator()(RobotState s1, RobotState s2);
};

