#ifndef GRAPH_MAP_PATH_H_
#define GRAPH_MAP_PATH_H_

#include <stack>
#include <string>

#include "Node.h"

namespace graph_map
{

class Path: public std::stack<graph_map::Node*>
{
public:
    std::string toString();
};

}

#endif
