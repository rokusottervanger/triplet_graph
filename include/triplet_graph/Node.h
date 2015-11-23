#ifndef TRIPLET_GRAPH_NODE_H_
#define TRIPLET_GRAPH_NODE_H_

#include <vector>
#include <string>
#include <map>

namespace triplet_graph
{

class Node
{
    friend class Graph;
public:
    Node(std::string id = generateId() ):id(id){}

    static std::string generateId();

    std::string id;

    std::vector<int> edges; // indices to vector of edge2s (edges_) in graph
    std::vector<int> triplets; // indices to vector of edge3s (triplets_) in graph

    int edgeByPeer(const int peer) const;
    int peerByEdge(const int edge) const;
    std::vector<int> tripletsByPeer(const int peer) const;

    void addTriplet(const int triplet);
    void addEdge(const int edge);

// TODO:
//    void deleteTriplet(const int triplet);
//    void deleteEdge(const int edge);

private:

    // TODO: implement filling these maps!!!
    std::map<int,int> peer_by_edge_; // map from edges to the node indices of the peer
    std::map<int,int> edge_by_peer_; // map from connected node indices to respective edges
    std::map<int,std::vector<int> > triplets_by_peer_; // map from connected node indices to common triplets

    std::vector<int> deleted_edges_;
    std::vector<int> deleted_triplets_;
};

}

#endif
