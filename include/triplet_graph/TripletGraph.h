#ifndef GRAPH_MAP_GRAPH_H_
#define GRAPH_MAP_GRAPH_H_

#include <list>
#include <geolib/datatypes.h>
#include <tue/config/configuration.h>
#include <triplet_graph/Measurement.h>

#include "TripletEdge.h"
#include "TripletNode.h"
#include "TripletPath.h"

namespace triplet_graph
{

class TripletGraph
{
public:
    TripletGraph(){}

    ~TripletGraph(){}

//    TripletNode* addNode(std::string id);

//    TripletNode* addNode(const Node &node);

    Node* addNode(std::string id, Node* n_1, Node* n_2, double side13, double side23);

    Path Dijkstra(Node *n1, Node *n2);

    bool configure(tue::Configuration &config);

    Node* findNodeByID(std::string id);

    void update(Measurements);

protected:
    std::list<Node> nodes_;
    std::list<Triplet> edges_;

};

}

#endif
