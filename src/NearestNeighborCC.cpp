#include "triplet_graph/NearestNeighborCC.h"

#include "triplet_graph/Graph.h"
#include "triplet_graph/Measurement.h"

#include <geolib/math_types.h>

#ifndef PI
#define PI 3.14159265
#endif

namespace triplet_graph
{

double prob(double a, double b)
{
    /* Algorithm for computing the probability of a under a zero-
     * centered normal distribution with standard deviation b.
     */
    double b_sq = b*b;
    return exp(-0.5*a*a/b_sq) / sqrt(2*PI*b_sq);
}

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

    // TODO: better incorporation of odom error!
    return (cur_measurement_pt - graph_positions.measurement.points[node_index]).length2() / ( cur_measurement_std_dev_sq + odom_std_dev_sq );
}

}
