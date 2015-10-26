#include "triplet_graph/Node.h"

namespace triplet_graph
{

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

// TODO:
//    void deleteTriplet(const int triplet);
//    void deleteEdge(const int edge);

}
