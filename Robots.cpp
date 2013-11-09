
#include "Robots.hpp"

#include <cmath>

const unsigned int NUMBER_OF_POINTS_ON_UNIT_CIRCLE = 16;
const double PI = 3.141592653589793238462;
const double TWO_PI_64 = (2.0*PI)/NUMBER_OF_POINTS_ON_UNIT_CIRCLE;

std::vector<Vector2D> accLookUp() 
{
    double theta = 0.0;
    double c = 0.0;
    double s = 0.0;
    std::vector<Vector2D> result;

    for(int i = 0; i < NUMBER_OF_POINTS_ON_UNIT_CIRCLE; i++)
    {
        theta = i*TWO_PI_64;
        c = std::cos(theta);
        s = std::sin(theta);

        result.push_back(Vector2D(c,s));
    }

    result.push_back(Vector2D(0.0,0.0));

    return result;
}

const std::vector<Vector2D> ACC_LOOKUP = accLookUp();

std::ostream & operator<<(std::ostream & os, const RobotState & v)
{
    os << "Position: " << v.pos << std::endl;
    os << "Speed: " << v.speed << std::endl;
    os << "Elapsed Time: " << v.elapsedTime << std::endl;
    os << std::endl;

    return os;
}

Enemy Enemy::getAfterTime(unsigned int t)
{
    int index = 0;
    int trajSize = trajectory.size();
    if(closedTrajectory)
    {
        index = t % trajSize;
    }
    else
    {
        index = t % (2 * trajSize);

        if(index >= trajectory.size())
        {
            index = trajSize - index;
        }
    }

    Vector2D newPos = trajectory[index];

    Enemy newEnemy;
    newEnemy.circ = Circle(newPos,this->circ.r());
    newEnemy.trajectory = this->trajectory;
    newEnemy.closedTrajectory = this->closedTrajectory;

    return newEnemy;
}

Environment Environment::getAfterTime(unsigned int t)
{
    Environment newEnv;
    newEnv.boundary = this->boundary;
    newEnv.obstacles = this->obstacles;
    std::vector<Enemy> newEnemies;
    for(auto e : this->enemies)
    {
        newEnemies.push_back(e.getAfterTime(t));
    }
    newEnv.enemies = newEnemies;

    return newEnv;
}

bool Environment::isSound(const Circle & circ) const
{
    if(!boundary.completelyContains(circ))
    {
        return false;
    }

    for(auto obs : obstacles)
    {
        if(obs.collidesWith(circ))
        {
            return false;
        }
    }

    for(auto e : enemies)
    {
        if(e.circ.collidesWith(circ))
        {
            return false;
        }
    }

    return true;
}

size_t RobotHash::operator()(RobotState s)
{
    return (size_t)(s.elapsedTime + s.pos.x() + s.pos.y() + s.speed.x() + s.speed.y());
}

const double RobotCompare::EPSILON = 0.00001;

bool RobotCompare::operator()(RobotState s1, RobotState s2)
{
    double dPos = (s1.pos.sub(s2.pos)).length();
    double dSpeed = (s1.speed.sub(s2.speed)).length();

    bool posOK = (dPos < EPSILON) && (dPos > -EPSILON);
    bool speedOK = (dSpeed < EPSILON) && (dSpeed > -EPSILON);

    return posOK && speedOK;
}

double RobotEdgeLength::operator()(RobotState s1, RobotState s2)
{
    return 1.0;
}

double RobotHeuristic::operator()(RobotState s1, RobotState s2)
{
    double distance = (s1.pos.sub(s2.pos)).length();

    return distance/(s1.speed.length());
    //return distance/constraints.maxV;
}

std::vector<Vector2D> accs(double scale)
{
    std::vector<Vector2D> result;

    for(auto a : ACC_LOOKUP)
    {
        result.push_back(a.mult(scale));
    }

    return result;
}

std::vector<RobotState> RobotNeighbours::operator()(RobotState s)
{
    static std::vector<Vector2D> lookUp = accs(constraints.maxAcc);
    std::vector<RobotState> result;


    Vector2D newPos = s.pos.add(s.speed);
    unsigned int newTime = s.elapsedTime + 1;
    Environment currentEnv = env.getAfterTime(newTime);

    if(!currentEnv.isSound(Circle(newPos,constraints.r)))
    {
        return result;
    }

    for(auto dV : lookUp)
    {
        Vector2D newV = s.speed.add(dV);
        if(newV.length() <= constraints.maxV)
        {
            RobotState newState;
            newState.pos = newPos;
            newState.speed = newV;
            newState.elapsedTime = newTime;

            result.push_back(newState);
        }
    }

    return result;
}

