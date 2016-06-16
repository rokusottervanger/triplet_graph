#ifndef TRIPLET_GRAPH_NEAREST_NEIGHBOR_PC_H_
#define TRIPLET_GRAPH_NEAREST_NEIGHBOR_PC_H_

#include "ProbabilityCalculator.h"

namespace triplet_graph
{

class NearestNeighborPC : public ProbabilityCalculator
{
public:
    double calculateProbability(const Graph& graph,
                                const geo::Vec3d& cur_measurement_pt,
                                const double cur_measurement_std_dev,
                                const OdomModel& odom_model,
                                const AssociatedMeasurement& graph_positions,
                                const int node_index,
                                const AssociatedMeasurement& input_associations,
                                const Path &path) const;

};

}

#endif
