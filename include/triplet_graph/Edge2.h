#ifndef TRIPLET_GRAPH_EDGE2_H_
#define TRIPLET_GRAPH_EDGE2_H_

#include <map>

namespace triplet_graph
{

struct Edge2
{
    friend class Graph;
public:
    Edge2(const int node1, const int node2, const double length, const double stddev):
        A(node1), B(node2), l(length), deleted(false), std_dev(stddev) {}

    // Two edges are equal if they connect the same nodes
    inline bool operator== (Edge2 e) { return e.A == A && e.B == B || e.A == B && e.B == A; }

    int A, B; // node indices

    double l; // length (nominal value)
    double std_dev; // standard deviation in length

    bool rigid; // denotes whether or not the distance is fixed in the real world (connected nodes are part of the same rigid object)

    bool deleted;

    int getOtherNode(const int n) const;
    int tripletByNode(const int n) const;

private:

    std::map< int, int> triplet_by_node_; // map from node indices the shared triplet of the three nodes
};

}

#endif
