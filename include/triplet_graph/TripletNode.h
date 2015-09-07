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

    std::vector<Triplet*> triplets;
};

}

#endif
