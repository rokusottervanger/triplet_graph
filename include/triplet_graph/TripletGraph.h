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

    int addNode(const std::string &id);

    int addEdge(const int&, const int&, const int&, double &, double &);

    Path Dijkstra(const int& n1, const int& n2);

    bool configure(tue::Configuration &config);

    int findNodeByID(const std::string& id);

    void update(const Measurements& measurements);

protected:
    std::vector<Node> nodes_;
    std::vector<Triplet> edges_;

private:
    std::vector<int> deleted_nodes_;
    std::vector<int> deleted_edges_;
};

}

#endif
