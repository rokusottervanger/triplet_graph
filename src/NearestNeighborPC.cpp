#include "triplet_graph/NearestNeighborPC.h"

#include "triplet_graph/Graph.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/OdomModel.h"

#include <geolib/math_types.h>

namespace triplet_graph
{

double NearestNeighborPC::calculateProbability(const Graph& graph,
                                        const geo::Vec3d& cur_measurement_pt,
                                        const double cur_measurement_std_dev,
                                        const OdomModel& odom_model,
                                        const AssociatedMeasurement& graph_positions,
                                        const int node_index,
                                        const AssociatedMeasurement& input_associations,
                                        const Path& path) const
{
    double cur_measurement_std_dev_sq = cur_measurement_std_dev * cur_measurement_std_dev;

    // TODO: better incorporation of odom error!
    return exp(-(cur_measurement_pt - graph_positions.measurement.points[node_index]).length2() / ( 2 * cur_measurement_std_dev_sq ));
}

}
