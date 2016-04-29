#ifndef TRIPLET_GRAPH_NEAREST_NEIGHBOR_CC_H_
#define TRIPLET_GRAPH_NEAREST_NEIGHBOR_CC_H_

#include "CostCalculator.h"

namespace triplet_graph
{

class NearestNeighborCC : public CostCalculator
{
public:
//    NearestNeighborCC();

    double calculateCost(const Graph& graph,
                         const geo::Vec3d& cur_measurement_pt,
                         const double cur_measurement_std_dev,
                         const AssociatedMeasurement& graph_positions,
                         const int node_index,
                         const AssociatedMeasurement& input_associations,
                         Path& path) const;

};

}

#endif
