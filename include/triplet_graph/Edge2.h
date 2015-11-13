#ifndef TRIPLET_GRAPH_EDGE2_H_
#define TRIPLET_GRAPH_EDGE2_H_

#include <map>

namespace triplet_graph
{

struct Edge2
{
public:
    Edge2(const int& node1, const int& node2, const double& length):
        A(node1), B(node2), l(length), deleted(false){}

    // Two edges are equal if they connect the same nodes
    inline bool operator== (Edge2 e) { return e.A == A && e.B == B || e.A == B && e.B == A; }

    int A, B; // node indices
    double l; // length

    std::map< int, int> triplet_by_node; // map from node indices the shared triplet of the three nodes

    bool deleted;

    // TODO: Uncertainty
};

}

#endif
