#ifndef GRAPH_MAP_TRIPLET_EDGE_H_
#define GRAPH_MAP_TRIPLET_EDGE_H_

#include <geolib/datatypes.h>

#include "TripletRelation.h"

namespace triplet_graph
{

struct Node;

// -----------------------------------------------------------------------------------------------

class Triplet
{
public:
    Triplet(Node* n_1, Node* n_2, Node* n_3, double &a, double&b):
        A_(n_1), B_(n_2), C_(n_3), a_(a), b_(b) {}

    Triplet(Node* n_1, Node* n_2, Node* n_3, double &a, double &b, double &c):
        A_(n_1), B_(n_2), C_(n_3), a_(a), b_(b), c_(c) {}

    // TODO: Equality operator
    // Two edges are equal if they connect the same nodes
//    inline bool operator== (TripleEdge e) { return e.n1 == n1 && e.n2 == n2 || e.n1 == n2 && e.n2 == n1; }

private:
    Node *A_, *B_, *C_;
    double a_, b_, c_;

    /*
     *         C
     *        /  \
     *       b    `a
     *      /       `\
     *     A ----c---- B
     */

    // There are several ways to uniquely define a triangle. If A is a fixed angle and S is a fixed side:
    // ASA
    // SAS
    // SSS
    // AAS

    // TODO: make incompleteness of pose relationship possible
//    TripleRelation ir; // Triple, not done yet.

    double w; // Edge weight
};

}

#endif
