
#include "GridState.hpp"

struct Hash
{
    size_t operator()(GridState s)
    {
        return (size_t)(s.x + s.y);
    }
};

std::function<size_t(GridState)> GridState::hash()
{
    Hash h;

    return h;
}

struct Compare
{
    bool operator()(GridState s1, GridState s2)
    {
        return ((s1.x == s2.x) && (s1.y == s2.y));
    }
};

std::function<bool(GridState,GridState)> GridState::compare()
{
    Compare cmp;

    return cmp;
}

struct Heuristic
{
    double operator()(GridState s1, GridState s2)
    {
        int dx = s1.x - s2.x;
        int dy = s1.y - s2.y;

        dx = (dx >= 0) ? dx : -dx;
        dy = (dy >= 0) ? dy : -dy;

        return (double)(dx + dy);
    }
};

std::function<double(GridState,GridState)> GridState::heuristic()
{
    Heuristic h;

    return h;
}

struct EdgeLength
{
    double operator()(GridState s1, GridState s2)
    {
        return 1.0;
    }
};

std::function<double(GridState,GridState)> GridState::edgeLength()
{
    EdgeLength l;

    return l;
}

bool check(int x, int y)
{
    bool checkHigh = (x < GridState::DIMENSION) && (y < GridState::DIMENSION);
    bool checkLow = (x >= 0) && (y >= 0);

    return (checkHigh && checkLow);
}

void add(int x, int y, std::vector<GridState> & v)
{
    if(check(x,y))
    {
        GridState s;
        s.x = x;
        s.y = y;
        v.push_back(s);
    }
}

struct Neighbours
{
    std::vector<GridState> operator()(GridState s)
    {
        std::vector<GridState> result;

        int newX = s.x + 1;
        int newY = s.y;

        add(newX,newY,result);

        newX = s.x - 1;
        add(newX,newY,result);

        newX = s.x;
        newY = s.y + 1;
        add(newX,newY,result);

        newY = s.y - 1;
        add(newX,newY,result);


        return result;
    }
};

std::function<std::vector<GridState>(GridState) > GridState::neighbours()
{
    Neighbours n;

    return n;
}

