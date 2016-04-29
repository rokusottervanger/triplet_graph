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

bool configure(Graph &g, tue::Configuration &config);
bool load(Graph &g, std::string filename);

void setRigidEdges(Graph &graph, const std::vector<int>& nodes);

void calculatePositions(const Graph& graph, const Path& path, AssociatedMeasurement& positions);

void associate( const Graph &graph,
                const Measurement &measurement,
                AssociatedMeasurement &associations,
                Measurement &unassociated,
                const int goal_node_i,
                tue::Configuration& config);

void associate( const Graph &graph,
                const Measurement &measurement,
                AssociatedMeasurement &associations,
                Measurement &unassociated,
                const int goal_node_i,
                Path& path,
                tue::Configuration& config);

void updateGraph(Graph &graph, const AssociatedMeasurement &associations, bool update_lengths=false);

void extendGraph(Graph &graph, const Measurement &unassociated, AssociatedMeasurement &associations, const double edge_std_dev);

AssociatedMeasurement generateVisualization(const Graph& graph, const AssociatedMeasurement& associations, Path &path);

void save(const Graph &graph, const std::string &filename);


}


#endif
