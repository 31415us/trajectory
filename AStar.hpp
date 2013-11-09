
#pragma once

#include <vector>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>

#include "PriorityQueue.hpp"

template<class T>
std::vector<T> aStar(
        T start, 
        T goal, 
        std::function<double(T,T)> heuristic, 
        std::function<std::vector<T>(T)> neighbours, 
        std::function<double(T,T)> edgeLength,
        std::function<size_t(T)> hash,
        std::function<bool(T,T)> compare
        )
{
    std::unordered_set<T,std::function<size_t(T)>,std::function<bool(T,T)> > visited(0,hash,compare);
    std::unordered_map<T,T,std::function<size_t(T)>,std::function<bool(T,T)> > parent(0,hash,compare);
    std::unordered_map<T,double,std::function<size_t(T)>,std::function<bool(T,T)> > distance_travelled(0,hash,compare);
    PriorityQueue<T> queue(hash,compare);

    visited.insert(start);
    distance_travelled[start] = 0.0;

    std::vector<T> result;

    T current = start;
    while(! compare(current,goal))
    {
        std::cout << "currently processing:" << std::endl;
        std::cout << current << std::endl;
        // get neighbours of current state and filter out states already visited
        std::vector<T> neigh;
        for(auto n : neighbours(current))
        {
            if(visited.find(n) == visited.end())
            {
                neigh.push_back(n);
            }
        }


        double current_dist = distance_travelled[current];
        for(auto n : neigh)
        {
            double dist = current_dist + edgeLength(current,n);
            queue.update(n,dist + heuristic(n,goal));
            distance_travelled[n] = dist;
            parent[n] = current;
            visited.insert(n);
        }

        current = queue.poll();
    }

    // backtrack path
    while(parent.find(current) != parent.end())
    {
        result.push_back(current);
        current = parent[current];
    }

    std::reverse(result.begin(),result.end());

    return result;
}

