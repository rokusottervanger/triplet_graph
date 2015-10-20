#ifndef TRIPLET_GRAPH_NODE_H_
#define TRIPLET_GRAPH_NODE_H_

#include <vector>
#include "Edge2.h"
#include "Edge3.h"

namespace triplet_graph
{

class Node
{
public:
    Node(){}

    Node(std::string id):id(id){}

    std::string id;

    std::vector<int> edges; // Edges to parent nodes
    std::vector<int> triplets; // Edges to child nodes

    void addTriplet(int triplet)
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

    void addEdge(int edge)
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

    // TODO: methods to remove edges and triplets from node
    // When a node is removed, al edges to it must also be removed from peers

private:

    std::vector<int> deleted_edges_;
    std::vector<int> deleted_triplets_;
};

}

#endif
