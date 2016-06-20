#include "triplet_graph/PathFinder.h"
#include <iostream>

#include <queue>
#include "triplet_graph/Path.h"

namespace triplet_graph
{

PathFinder::PathFinder(const Graph &graph, const std::vector<int> &source_nodes):
    graph_(&graph),
    all_done_(false)
{
    prevs_ = std::vector<int>(graph.size(),-1);  // vector of edges from which each node is reached (index is node index)
    ns_ = std::vector<double>(graph.size(),1e38); // cost to get to nodes (index is node index)
    visited_nodes_ = std::vector<bool>(graph_->size(),false);

    for ( std::vector<int>::const_iterator it = source_nodes.begin(); it != source_nodes.end(); ++it )
        source_nodes_.insert(*it);
}

// -----------------------------------------------------------------------------------------------

double PathFinder::findPath(Path &path)
{
    if ( all_done_ )
    {
        tracePath(-1,path);
        return 0;
    }
    else
        return findPath(-1, path);
}

// -----------------------------------------------------------------------------------------------

double PathFinder::weighting(double l3, double l1, double l2)
/** Weighting function for triangles. Uses the lengths of the triangle's edges
 * to calculate the cost of using this triangle to calculate the position of
 * the child node. The first length connects the parent nodes, the other two
 * connect the child node to its respective parents.
 * This function uses the error propagation formula for propagation of errors
 * in the edges to the error in the position of the node.
 * TODO: Also use the std dev of the edges in this calculation.
 */
{
    double l1_sq = l1*l1;
    double l2_sq = l2*l2;
    double l3_sq = l3*l3;

    double x = (l1_sq-l2_sq+l3_sq)/(2*l3);
    double x_sq = x*x;

    double dx_sq = l1_sq/l3_sq + l2_sq/l3_sq + 1/4.0;
    double thing = 2 * l1 - 2*(l1/l3)*x; // TODO naming?
    double dy_sq = 1/(l1_sq-x_sq) * thing*thing + (4*l2_sq*x_sq)/(l3_sq*(l1_sq-x_sq))+x_sq/(l1_sq-x_sq);
    return dx_sq+dy_sq;
}

// -----------------------------------------------------------------------------------------------

double PathFinder::findPath(const int target_node, Path& path)
{
    // If path was already calculated...
    if ( target_node != -1 && visited_nodes_[target_node] )
    {
        // directly trace back that path
        tracePath(target_node, path);
        return ns_[target_node];
    }

    /*
     * The priority queue holds unvisited nodes to which a cost has been calculated, sorted by
     * the total cost to get to that node. The cost of the unvisited node with the lowest cost
     * is guaranteed to be the lowest possible cost.
     */
    std::priority_queue<CostInt, std::vector<CostInt>, std::greater<CostInt> > Q;

    // Mark all source nodes visited and add them to the queue
    std::cout << "Source nodes: " << std::endl;
    for ( std::set<int>::const_iterator it = source_nodes_.begin(); it != source_nodes_.end(); ++it )
    {
        std::cout << *it << ", ";
        if ( *it == -1 )
             std::cout << "[FIND_PATH] Warning! Input node index is -1!" << std::endl;
        else
        {
            Q.push(CostInt(0.0, *it));
            visited_nodes_[*it] = true;
            ns_[*it] = 0;
        }
    }
    std::cout << std::endl;

    if ( Q.size() < 1 )
    {
         std::cout << "\033[31m" << "[PathFinder] Not enough valid input points!" << "\033[0m" << std::endl;
    }


    /* The path is to contain the series of nodes to get from the source nodes to
     * the target node. To construct this path, the prevs vector is maintained,
     * holding for every node the set of nodes that its current best calculated
     * cost originates from. Nodes for which no cost is calculated yet, and source
     * nodes have -1 at their places in prevs.
     */
    while(!Q.empty())
    {
        // Take the cheapest node from the queue (guaranteed to have lowest cost calculated)
        int u = Q.top().second; // current edge, cheapest pair of nodes so far

        Q.pop();
        visited_nodes_[u] = true;

        // If current node is the target node, trace back path
        if ( target_node != -1 && u == target_node )
        {
            tracePath(target_node, path);

            // When finished, return cost to target node
            return ns_[target_node];
        }

        Graph::const_iterator node_it = graph_->begin() + u;

        // Go through current node's edges
        for ( std::vector<int>::const_iterator e_it = node_it->edges.begin(); e_it != node_it->edges.end(); ++e_it )
        {
            Graph::const_edge2_iterator edge_it = graph_->beginEdges() + *e_it;

            int peer = edge_it->getOtherNode(u);

            // Only consider edges that connect current node to visited nodes
            if ( !visited_nodes_[peer] )
                continue;

            std::vector<int> common_triplets = (graph_->begin() + edge_it->A)->tripletsByPeer(edge_it->B);

            // Run through common triplets of the current pair of nodes
            for ( std::vector<int>::iterator t_it = common_triplets.begin(); t_it != common_triplets.end(); ++t_it )
            {
                // Retrieve the right node from the triplet.
                Graph::const_edge3_iterator trip_it = graph_->beginTriplets() + *t_it;

                int v = trip_it->getThirdNode(edge_it->A,edge_it->B);

                if ( visited_nodes_[v] )
                    continue;

                Graph::const_iterator node_it = graph_->begin() + v;

                double w = 0;
                double l3 = edge_it->l;                                                     // Edge between parents
                double l1 = (graph_->beginEdges() + node_it->edgeByPeer(edge_it->A))->l;    // Edge between parent A and current node
                double l2 = (graph_->beginEdges() + node_it->edgeByPeer(edge_it->B))->l;    // Edge between parent B and current node

                // Check triangle inequality!
                double p  = ( l1 + l2 + l3 )/2.0;
                double x_sq = 0;
                double y = 0;
                double y_sq = 0;
                if ( (p-l1)*(p-l2)*(p-l3) > 0 )
                {
                    double dl1 = (graph_->beginEdges() + node_it->edgeByPeer(edge_it->A))->std_dev;
                    double dl2 = (graph_->beginEdges() + node_it->edgeByPeer(edge_it->B))->std_dev;
                    double dl3 = edge_it->std_dev;

                    double dl1_sq = dl1*dl1;
                    double dl2_sq = dl2*dl2;
                    double dl3_sq = dl3*dl3;

                    double l1_sq = l1*l1;
                    double l2_sq = l2*l2;
                    double l3_sq = l3*l3;

                    double x = (l1_sq-l2_sq+l3_sq)/(2*l3);
                    x_sq = x*x;

                    y = sqrt(l1_sq - x_sq);
                    y_sq = y*y;

                    double dx_sq = l1_sq/l3_sq * dl1_sq + l2_sq/l3_sq * dl2_sq + dl3_sq/4.0 ;
                    double thing = 2 * l1 - 2*(l1/l3)*x; // TODO naming?
                    double dy_sq = 1/(l1_sq-x_sq) * thing*thing * dl1_sq + (4*l2_sq*x_sq)/(l3_sq*(l1_sq-x_sq)) *dl2_sq + x_sq/(l1_sq-x_sq) * dl3_sq;

                    w = dx_sq+dy_sq;
                }

                double dp_1_sq = ns_[edge_it->A];
                double dp_2_sq = ns_[edge_it->B];

                double l3_sq = l3*l3;

                double x_scaled_sq = x_sq/l3_sq;
                double y_scaled_sq = y_sq/l3_sq;

                double new_cost = dp_1_sq + y_scaled_sq * ( dp_1_sq + dp_2_sq ) + (y/l3 + 1)*(y/l3 + 1) * dp_1_sq + x_scaled_sq * dp_2_sq + w;

                // If this new cost ends up to be lower than the cost to one of the parents,
                if ( dp_1_sq > new_cost || dp_2_sq > new_cost )
                    new_cost = std::max(dp_1_sq,dp_2_sq);

                // If path to third node is cheaper than before, update cost to that node, add the cheapest connecting edge to priority queue
                // of potential nodes to visit and record what the previous node was.
                if (ns_[v] > new_cost)
                {
                    ns_[v] = new_cost;

                    Q.push(CostInt(new_cost, v));

                    // Store edge that lead to this node
                    prevs_[v] = *e_it;
                }
            }
        }
    }

//    std::cout << "ns_: \n[ " << std::endl;
//    for ( int i = 0; i < ns_.size(); i++ )
//    {
//        std::cout << i << ": " << ns_[i] << ", ";
//    }
//    std::cout << std::endl;

//    std::cout << "Prevs: \n[ " << std::endl;
////    for ( std::vector<int>::const_iterator it = prevs_.begin(); it != prevs_.end(); ++it )
//    for ( int i = 0; i < prevs_.size(); i++ )
//    {
//        std::cout << i << ": (" << (graph_->beginEdges() + prevs_[i])->A << ", " << (graph_->beginEdges() + prevs_[i])->B << "), ";
//    }
//    std::cout << std::endl;

    // If there is no target node (target_node == -1), the program will get here after calculating paths to ever node in the graph.
    // Now push all nodes in the graph into the path.
    tracePath(target_node,path);
    all_done_ = true;

    return 0;
}

// -----------------------------------------------------------------------------------------------

void PathFinder::tracePath(const int target_node, Path& path)
{
    std::priority_queue<CostInt, std::vector<CostInt>, std::less<CostInt> > trace;

    bool add_all;

    // Push target node into trace
    if ( target_node > -1 )
    {
        add_all = false;
        trace.push(CostInt(ns_[target_node],target_node));
    }
    else
    {
        add_all = true;
        for ( int i = 0; i < ns_.size(); ++i )
        {
            if ( ns_[i] != 0 )
                trace.push(CostInt(ns_[i],i));
        }
    }
    int n, e;

    std::vector<int> es = prevs_;

    while ( !trace.empty() )
    {
        n = trace.top().second;
        trace.pop();

        // The current node refers to its originating edge using prevs
        e = prevs_[n];
        int visited = es[n];

        // If edge not yet visited and pushed to path, push both node A and node B of this edge to trace and push current node to path
        if ( visited != -1 )
        {
            // Don't add source nodes (cost == 0) to trace
            Graph::const_edge2_iterator edge_it = graph_->beginEdges() + e;
            if ( edge_it->deleted )
            {
                std::cout << "[FIND_PATH] Warning! Skipping deleted edge" << std::endl;
                continue;
            }

            int na = edge_it->A;
            if ( ns_[na] != 0 && !add_all )
                trace.push(CostInt(ns_[na],na));

            int nb = edge_it->B;
            if ( ns_[nb] != 0 && !add_all )
                trace.push(CostInt(ns_[nb],nb));

            path.node_indices[n] = path.size();
            path.push_back(n);
            path.parent_tree.push_back(std::make_pair(na,nb));
            path.costs.push_back(ns_[n]);

            es[n] = -1;
        }
    }

    // Finally, add source nodes to path
    std::vector<int> source_nodes_vec;
    source_nodes_vec.insert( source_nodes_vec.end(), source_nodes_.begin(), source_nodes_.end() );
    for ( int i = 0; i < source_nodes_vec.size(); i++ )
    {
        path.node_indices[source_nodes_vec[i]] = path.size() + i;
    }
    path.insert(path.end(), source_nodes_.begin(), source_nodes_.end());
    path.parent_tree.resize(path.size(),std::make_pair(-1,-1));
    path.costs.resize(path.size(),0.0);

}

}
