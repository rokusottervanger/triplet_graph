#ifndef TRIPLET_GRAPH_NODE_H_
#define TRIPLET_GRAPH_NODE_H_

#include <vector>
#include <string>
#include <map>

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

    std::map<int,int> peer_by_edge; // map from edges to the node indices of the peer
    std::map<int,int> edge_by_peer; // map from connected node indices to respective edges

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
