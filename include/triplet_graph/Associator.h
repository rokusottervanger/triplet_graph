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

    bool configure();
    double associate(const Measurement &graph_positions, const Measurement &measurement, AssociatedMeasurement& resulting_associations);

    void setAssociations(const AssociatedMeasurement& associations);
    void setGraph(const Graph& graph);

    bool getAssociations(const Measurement &measurement, AssociatedMeasurement& associations );
    bool getUnassociatedPoints( Measurement& unassociated_points );
    bool getPath(Path& path);

private:
    Measurement unassociated_points_;
    AssociatedMeasurement associations_;
    const Graph* graph_ptr_;
    Path path_;

    bool associated_;
    double max_association_dist_;
    double max_association_dist_sq_;

    void nearestNeighbor( const Measurement& measurement, const std::vector<geo::Vec3d> prediction );
    Graph getObjectSubgraph(const Graph &graph, const int node_i );


};

}

#endif
