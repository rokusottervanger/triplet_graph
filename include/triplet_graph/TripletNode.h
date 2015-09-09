#ifndef GRAPH_MAP_NODE_H_
#define GRAPH_MAP_NODE_H_

#include <vector>
#include "TripletEdge.h"

namespace triplet_graph
{

struct Node
{    
    Node(std::string id):id(id){}

    std::string id;

    std::vector<int> parent_edges; // Edges to parent nodes
    std::vector<int> child_edges; // Edges to child nodes
};

}

#endif
