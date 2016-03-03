#ifndef TRIPLET_GRAPH_ASSOCIATOR_H_
#define TRIPLET_GRAPH_ASSOCIATOR_H_

#include <tue/config/configuration.h>

#include "triplet_graph/graph_types.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/Path.h"
#include "triplet_graph/Graph.h"

namespace triplet_graph
{

class Associator
{
public:
    Associator();

    bool configure(tue::Configuration &config);
    double associate(const Graph &graph, const Measurement &measurement);

    void setAssociations(const AssociatedMeasurement& associations);
    void setGraph(const Graph& graph);

    bool getAssociations( AssociatedMeasurement& associations );
    bool getUnassociatedPoints( Measurement& unassociated_points );
    bool getPath(Path& path);

private:
    Measurement unassociated_points_;
    AssociatedMeasurement associations_;
    Graph graph_;
    Path path_;

    bool associated_;
    double max_association_dist_;

    void nearestNeighbor( const Measurement& measurement, const std::vector<geo::Vec3d> prediction );
    Graph getObjectSubgraph(const Graph &graph, const int node_i );


};

}

#endif
