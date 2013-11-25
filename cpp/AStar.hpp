
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
    std::unordered_map<T,double,std::function<size_t(T)>,std::function<bool(T,T)> > g_score(0,hash,compare);
    std::unordered_map<T,double,std::function<size_t(T)>,std::function<bool(T,T)> > f_score(0,hash,compare);
    PriorityQueue<T> queue(hash,compare);

    std::vector<T> result;

    g_score[start] = 0.0;
    f_score[start] = g_score[start] + heuristic(start,goal);

    queue.update(start,f_score[start]);


    
    while(!queue.isEmpty())
    {
        auto current = queue.poll();
        visited.insert(current);

        std::cout << "currently exploring:" << std::endl;

        std::cout << current << std::endl;

        if(compare(current,goal))
        {
            // backtrack path
            while(parent.find(current) != parent.end())
            {
                result.push_back(current);
                current = parent[current];
            }
            std::reverse(result.begin(),result.end());

            break;
        }

        for(auto n : neighbours(current))
        {
            double tentative_g_score = g_score[current] + edgeLength(current,n);
            double tentative_f_score = tentative_g_score + heuristic(n,goal);

            if(visited.find(n) != visited.end() && tentative_f_score >= f_score[n])
            {
                continue;
            }
            else
            {
                parent[n] = current;
                g_score[n] = tentative_g_score;
                f_score[n] = tentative_f_score;
                queue.update(n,f_score[n]);
            }
        }

    }

    return result;
}

