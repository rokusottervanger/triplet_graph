#ifndef TRIPLET_GRAPH_PATHFINDER_H_
#define TRIPLET_GRAPH_PATHFINDER_H_

#include "graph_types.h"
#include "Graph.h"
#include <set>
#include <vector>

namespace triplet_graph
{

class PathFinder
{
    const Graph* graph_;
    std::vector<int> prevs_;
    std::vector<double> ns_;
    std::set<int> source_nodes_;
    bool all_done_;

//    static const double inf = 1e38;
    typedef std::pair< double, int > CostInt; // First is a cost, second is an edge or a node index
    // In the initial graph search: first is the sum of the costs (so far) to get to the nodes connected by an edge, second is the respective edge index
    // In the path trace search: first is the cost to get to the node of which the index is second

    void tracePath(const int target_node, Path &path);

public:
    PathFinder(const Graph& graph, const std::vector<int>& source_nodes);

    double findPath(Path &path);
    double findPath(const int target_node, Path &path);
};

}

#endif
