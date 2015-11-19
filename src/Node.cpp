#include "triplet_graph/Node.h"
#include <iostream>
#include <stdlib.h>

namespace triplet_graph
{

std::string Node::generateId()
{
    static const char alphanum[] =
            "0123456789"
            "abcdef";

    std::string s;
    for (int i = 0; i < 16; ++i)
    {
        int n = rand() / (RAND_MAX / (sizeof(alphanum) - 1) + 1);
        s += alphanum[n];
    }

    return s;
}

void Node::addTriplet(const int triplet)
{
    if (deleted_triplets_.empty())
        triplets.push_back(triplet);
    else
    {
        int i = deleted_triplets_.back();
        triplets[i] = triplet;
        deleted_triplets_.pop_back();
    }
}

void Node::addEdge(const int edge)
{
    if (deleted_edges_.empty())
        edges.push_back(edge);
    else
    {
        int i = deleted_triplets_.back();
        triplets[i] = edge;
        deleted_triplets_.pop_back();
    }
}

int Node::edgeByPeer(const int peer) const
{
    std::map<int,int>::const_iterator it = edge_by_peer_.find(peer);
    if ( it != edge_by_peer_.end() )
    {
        return it->second;
    }
    std::cout << "[NODE] edgeByPeer: peer not found, returning -1" << std::endl;
    return -1;
}

// TODO:
//    void deleteTriplet(const int triplet);
//    void deleteEdge(const int edge);

}
