#ifndef TRIPLET_GRAPH_COST_CALCULATOR_H_
#define TRIPLET_GRAPH_COST_CALCULATOR_H_

#include <geolib/math_types.h>

namespace triplet_graph
{

class Graph;
class AssociatedMeasurement;
class Path;
class OdomModel;

class ProbabilityCalculator
{
public:
    virtual double calculateProbability(const Graph& graph,
                                        const geo::Vec3d& cur_measurement_pt,
                                        const double cur_measurement_std_dev,
                                        const OdomModel& odom_model,
                                        const AssociatedMeasurement& graph_positions,
                                        const int node_index,
                                        const AssociatedMeasurement& input_associations,
                                        const Path& path) const = 0;
};

}

#endif
