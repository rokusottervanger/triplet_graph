#ifndef GRAPH_MAP_PATH_H_
#define GRAPH_MAP_PATH_H_

#include <vector>
#include <string>
#include <iostream>

namespace triplet_graph
{

// TODO: Rename this. In graph theory this is not a path, but a directed subgraph with two sources and one sink.
class Path: public std::vector<int>
{
public:
    friend std::ostream& operator<<(std::ostream& os, const Path& path);

    std::vector< std::pair< int, int > > parent_tree;
    std::vector< double > costs;
};

}

#endif
