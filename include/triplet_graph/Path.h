#ifndef GRAPH_MAP_PATH_H_
#define GRAPH_MAP_PATH_H_

#include <stack>
#include <string>
#include <iostream>

namespace triplet_graph
{

class Path: public std::stack<int>
{
public:
    friend std::ostream& operator<<(std::ostream& os, Path dt);
};

}

#endif
