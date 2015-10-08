#ifndef TRIPLET_GRAPH_EDGE3_H_
#define TRIPLET_GRAPH_EDGE3_H_

namespace triplet_graph
{

class Edge3
{
public:
    Edge3(const int& node_1, const int& node_2, const int& node_3):
        A(node_1), B(node_2), C(node_3) {}

    // TODO: Equality operator
    // Two triplets are equal if they connect the same nodes
//    inline bool operator== (Edge3 e) { return e.A == A && e.B == B || e.A == B && e.B == A; }

    // Nodes in clockwise order
    int A, B, C;
};

}

#endif
