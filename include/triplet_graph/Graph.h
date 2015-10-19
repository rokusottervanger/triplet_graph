#ifndef TRIPLET_GRAPH_GRAPH_H_
#define TRIPLET_GRAPH_GRAPH_H_

#include <list>
#include <geolib/datatypes.h>
#include <tue/config/configuration.h>
#include <triplet_graph/Measurement.h>

#include "Edge2.h"
#include "Edge3.h"
#include "Node.h"

namespace triplet_graph
{

class Graph
{
public:
    Graph(){}

    ~Graph(){}

    int addNode(const std::string &id);

    int addEdge2(const int&, const int&, double &);

    int addEdge3(const int&, const int&, const int&);

    void deleteNode(const int& i);

    // TODO: methods to remove edges

//    bool configure(tue::Configuration &config);

    void update(const Measurements& measurements);

    std::vector<Node> getNodes() const {return nodes_;}
    std::vector<Edge2> getEdge2s() const {return edges_;}
    std::vector<Edge3> getEdge3s() const {return triplets_;}

private:
    std::vector<Node> nodes_;
    std::vector<Edge2> edges_;
    std::vector<Edge3> triplets_;

    std::vector<int> deleted_nodes_;
    std::vector<int> deleted_edges_;
    std::vector<int> deleted_triplets_;
};

}

#endif
