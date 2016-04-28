#ifndef TRIPLET_GRAPH_EDGE_TENSION_CC_H_
#define TRIPLET_GRAPH_EDGE_TENSION_CC_H_

#include "CostCalculator.h"

namespace triplet_graph
{

class EdgeTensionCC : public CostCalculator
{
public:
//    EdgeTensionCC();

    double calculateCost(const Graph& graph , const geo::Vec3d& cur_measurement_pt, const double cur_measurement_std_dev, const AssociatedMeasurement& graph_positions, const int node_index, const AssociatedMeasurement& input_associations, Path& path);

private:
    geo::Vec3d getMostRecentNodePosition(const AssociatedMeasurement& associations, const AssociatedMeasurement& graph_positions, int node_i);
};

}

#endif
