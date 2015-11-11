#ifndef GRAPH_MAP_PATH_H_
#define GRAPH_MAP_PATH_H_

#include <vector>
#include <string>
#include <iostream>

namespace triplet_graph
{

class Path: public std::vector<int>
{
public:
    friend std::ostream& operator<<(std::ostream& os, const Path& path);
};

}

#endif
