
#include <stdlib.h>
#include <iostream>

#include <unordered_set>

#include "AStar.hpp"
#include "GridState.hpp"

class Obstacles
{
    private:
        bool _obsMap[GridState::DIMENSION * GridState::DIMENSION] = {0};

        bool check(int x, int y)
        {
            bool checkHi = (x < GridState::DIMENSION) && (x < GridState::DIMENSION);
            bool checkLo = (x >= 0) && (y >= 0);

            return checkHi && checkLo;
        }
    public:
        bool operator()(int x, int y)
        {
            if(check(x,y))
            {
                return _obsMap[y * GridState::DIMENSION + x];
            }
            else
            {
                return false;
            }
        }

        void add(int x, int y)
        {
            if(check(x,y))
            {
                _obsMap[y * GridState::DIMENSION + x] = true;
            }
        }
};

void printGrid(Obstacles obs, std::vector<GridState> p)
{
    std::unordered_set<GridState,std::function<size_t(GridState)>,std::function<bool(GridState,GridState)> > path(0,GridState::hash(),GridState::compare());

    for(auto s : p)
    {
        path.insert(s);
    }

    for(int i = 0; i < GridState::DIMENSION; i++)
    {
        for(int j = 0; j < GridState::DIMENSION; j++)
        {
            GridState s;
            s.x = i;
            s.y = j;

            if(obs(i,j))
            {
                std::cout << "x ";
            }
            else if(path.find(s) != path.end())
            {
                std::cout << "o ";
            }
            else
            {
                std::cout << "  ";
            }
        }

        std::cout << std::endl;
    }
}

class ObsNeigh
{
    private:
        Obstacles _obs;
    public:
        ObsNeigh(Obstacles obs) : _obs(obs) 
        {

        }
        std::vector<GridState> operator()(GridState state)
        {
            std::vector<GridState> n = GridState::neighbours()(state);
            std::vector<GridState> result;

            for(auto s : n)
            {
                if(! _obs(s.x,s.y))
                {
                    result.push_back(s);
                }
            }

            return result;
        }
};

int main(void)
{

    Obstacles obs;

    for(int i = 0; i < GridState::DIMENSION; i++)
    {
        if( i != (GridState::DIMENSION/2))
        {
            obs.add(i,i);
        }
    }

    ObsNeigh neigh(obs);

    GridState start;
    start.x = 0;
    start.y = GridState::DIMENSION - 1;
    GridState goal;
    goal.x = GridState::DIMENSION - 1;
    goal.y = 0;

    std::vector<GridState> path = aStar<GridState>(start,goal,GridState::heuristic(),neigh,GridState::edgeLength(),GridState::hash(),GridState::compare());

    printGrid(obs,path);

    return 0;
}

