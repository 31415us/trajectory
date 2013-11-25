
#pragma once

#include <functional>
#include <vector>


struct GridState
{
    int x;
    int y;
    static const int DIMENSION = 20;

    static std::function<size_t(GridState)> hash();
    static std::function<bool(GridState,GridState)> compare();
    static std::function<double(GridState,GridState)> heuristic();
    static std::function<double(GridState,GridState)> edgeLength();
    static std::function<std::vector<GridState>(GridState)> neighbours();

};

