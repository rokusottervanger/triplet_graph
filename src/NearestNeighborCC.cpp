#include "triplet_graph/NearestNeighborCC.h"

#include "triplet_graph/Graph.h"
#include "triplet_graph/Measurement.h"

#include <geolib/math_types.h>

namespace triplet_graph
{

double NearestNeighborCC::calculateCost(const Graph& graph,
                                        const geo::Vec3d& cur_measurement_pt,
                                        const double cur_measurement_std_dev,
                                        const double odom_std_dev,
                                        const AssociatedMeasurement& graph_positions,
                                        const int node_index,
                                        const AssociatedMeasurement& input_associations,
                                        Path& path) const
{
    double cur_measurement_std_dev_sq = cur_measurement_std_dev * cur_measurement_std_dev;
    double odom_std_dev_sq = odom_std_dev * odom_std_dev;

    return (cur_measurement_pt - graph_positions.measurement.points[node_index]).length2() / ( cur_measurement_std_dev_sq + odom_std_dev_sq );
}

}
