#ifndef TRIPLET_GRAPH_OPERATIONS_H_
#define TRIPLET_GRAPH_OPERATIONS_H_

#include "graph_types.h"
#include <string>
#include <vector>
#include <tue/config/configuration.h>
#include <geolib/datatypes.h>

namespace triplet_graph
{

int findNodeByID(const Graph& g, const std::string& id);

double findPath(const Graph &graph, const std::vector<int>& source_nodes, const int target_node, Path &path);

bool configure(Graph &g, tue::Configuration &config);

void calculatePositions(const Graph& graph, std::vector<geo::Vec3d>& positions, const Path& path);

void associate(const Graph &graph, const Measurement &measurement, AssociatedMeasurement &associations, const geo::Transform3d &delta, const int goal_node_i);

void associate(const Graph &graph, const Measurement &measurement, AssociatedMeasurement &associations, const geo::Transform3d &delta, const int goal_node_i, Path &path);

void updateGraph(Graph &graph, const AssociatedMeasurement &associations);

void extendGraph(Graph &graph, const Measurement &measurement, AssociatedMeasurement &associations);

void save(const Graph &graph, const std::string &filename);


}


#endif
