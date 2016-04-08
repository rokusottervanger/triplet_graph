#ifndef TRIPLET_GRAPH_EDGE3_H_
#define TRIPLET_GRAPH_EDGE3_H_

namespace triplet_graph
{

class Edge3
{
    friend class Graph;
public:
    Edge3(const int& node_1, const int& node_2, const int& node_3):
        A(node_1), B(node_2), C(node_3), deleted(false) {}

    // Two triplets are equal if they connect the same nodes
    inline bool operator== (Edge3 e) { return
                e.A == A && e.B == B && e.C == C ||
                e.A == B && e.B == C && e.C == A ||
                e.A == C && e.B == A && e.C == B; }

    // -----------------------------------------------------------------------------------------------

    inline int getThirdNode(const int node1, const int node2) const
    {
        if ( A == node1 && B == node2 )
            return C;
        else if ( B == node1 && C == node2 )
            return A;
        else if ( C == node1 && A == node2 )
            return B;
        else if ( A == node2 && B == node1 )
            return C;
        else if ( B == node2 && C == node1 )
            return A;
        else if ( C == node2 && A == node1 )
            return B;
        else
            return -1;
    }

    // -----------------------------------------------------------------------------------------------

    Edge3 flip()
    {
        Edge3 t = *this;
        t.A = B;
        t.B = A;
        return t;
    }

    // -----------------------------------------------------------------------------------------------

    // Nodes in counter-clockwise order
    int A, B, C;

    bool deleted;
};

}

#endif
