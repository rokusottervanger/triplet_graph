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

int getConnectingEdge2(const Graph &graph, const int Node1, const int Node2);

int getSecondNode(const Edge2& edge, const int node);

std::vector<int> getCommonTriplets(const Graph &graph, const int Node1, const int Node2);

int getThirdNode(const Edge3& triplet, const int node1, const int node2);

double findPath(const Graph &graph, const std::vector<int>& source_nodes, const int target_node, Path &path);

bool configure(Graph &g, tue::Configuration &config);

class AssociatedMeasurement;

void associate(Graph &graph, const Measurement &measurement, AssociatedMeasurement &associations, const geo::Transform3d &delta, const int goal_node_i);

void updateGraph(Graph &graph, const AssociatedMeasurement &associations);

void extendGraph(Graph &graph, const Measurement &measurement, AssociatedMeasurement &associations);

}


#endif
