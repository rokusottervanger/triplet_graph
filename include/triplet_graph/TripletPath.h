#ifndef GRAPH_MAP_PATH_H_
#define GRAPH_MAP_PATH_H_

#include <stack>
#include <string>

#include "triplet_graph/Node.h"

namespace triplet_graph
{

class Path: public std::stack<triplet_graph::Node*>
{
public:
    std::string toString();
};

}

#endif
