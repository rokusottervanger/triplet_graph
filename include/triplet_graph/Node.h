#ifndef TRIPLET_GRAPH_NODE_H_
#define TRIPLET_GRAPH_NODE_H_

#include <vector>
#include <string>

namespace triplet_graph
{

class Node
{
public:
    Node(){}

    Node(std::string id):id(id){}

    std::string id;

    std::vector<int> edges; // indices to vector of edge2s (edges_) in graph
    std::vector<int> triplets; // indices to vector of edge3s (triplets_) in graph

    void addTriplet(const int triplet);
    void addEdge(const int edge);

// TODO:
//    void deleteTriplet(const int triplet);
//    void deleteEdge(const int edge);

private:

    std::vector<int> deleted_edges_;
    std::vector<int> deleted_triplets_;
};

}

#endif
