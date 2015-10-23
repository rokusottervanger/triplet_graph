#ifndef GRAPH_MAP_PATH_H_
#define GRAPH_MAP_PATH_H_

#include <stack>
#include <string>
#include <iostream>

#include "triplet_graph/Node.h"

namespace triplet_graph
{

class Path: public std::stack<int>
{
public:
    std::string toString();

    friend std::ostream& operator<<(std::ostream& os, const Path& dt);
};

}

#endif
