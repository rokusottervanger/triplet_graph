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

double weighting1(double l_pp, double l_pc1, double l_pc2)
/* Weighting function for triangles. Uses the lengths of the triangle's edges
 * to calculate the cost of using this triangle to calculate the position of
 * the child node. The first length connects the parent nodes, the other two
 * connect the child node to its respective parents.
 * This function uses the ratio between the two child edge lengths and twice
 * the parent edge length.
 */
{
    double AR = ( l_pc1 + l_pc2 )/( 2*l_pp );
    return AR + 1/AR;
}

// -----------------------------------------------------------------------------------------------

double weighting2(double l_pp, double l_pc1, double l_pc2)
/* Weighting function for triangles. Uses the lengths of the triangle's edges
 * to calculate the cost of using this triangle to calculate the position of
 * the child node. The first length connects the parent nodes, the other two
 * connect the child node to its respective parents.
 * This function uses the ratio between the largest and smallest edge
 * lengths.
 */
{
    double skewness = std::max(std::max(l_pp,l_pc1),l_pc2)/std::min(std::min(l_pp,l_pc1),l_pc2);
    return skewness;
}

// -----------------------------------------------------------------------------------------------

// TODO: Check if i'm using the right lengths at the right places!!!!
double weighting3(double l3, double l1, double l2)
/* Weighting function for triangles. Uses the lengths of the triangle's edges
 * to calculate the cost of using this triangle to calculate the position of
 * the child node. The first length connects the parent nodes, the other two
 * connect the child node to its respective parents.
 * This function uses the ratio between the largest and smallest edge
 * lengths.
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
    if ( target_node != -1 && prevs_[target_node] != -1 )
    {
        // directly trace back that path
        std::cout << "Path was already calculated, so just tracing it back" << std::endl;
        tracePath(target_node, path);
        return ns_[target_node];
    }
    else
        std::cout << "Calculating path to all nodes" << std::endl;

    // get a copy of the nodes, edges and triplets in the graph
//    std::vector<Node>  nodes    = graph_->getNodes();
//    std::vector<Edge2> edges    = graph_->getEdge2s();
//    std::vector<Edge3> triplets = graph_->getEdge3s();

    // Track visited edges and triplets
    std::vector<double> es(graph_->numEdges(),1e38);

    /* The priority queue holds couples of nodes (edges) to be handled, sorted by
     * the sum of the cost to get to those nodes. The cost to get to the first
     * pair of nodes is obviously zero.
     */
    std::priority_queue<CostInt, std::vector<CostInt>, std::greater<CostInt> > Q;

    // Find all edges connecting the source nodes and add those edges to Q
    for ( std::set<int>::const_iterator it_1 = source_nodes_.begin(); it_1 != source_nodes_.end(); ++it_1 )
    {
        if ( *it_1 == -1 )
            std::cout << "[FIND_PATH] Warning! Input node index is -1!" << std::endl;
        else
        {
            for ( std::set<int>::const_iterator it_2 = source_nodes_.begin(); it_2 != it_1; ++it_2 )
            {
                if ( *it_2 == -1 )
                    std::cout << "[FIND_PATH] Warning! Input node index is -1!" << std::endl;
                else
                {
                    Graph::const_iterator node_it = graph_->begin() + *it_1;
                    if ( node_it->deleted )
                    {
                        std::cout << "[FIND_PATH] Warning! Skipping deleted node" << std::endl;
                        continue;
                    }

                    int edge = node_it->edgeByPeer(*it_2);
                    if ( edge == -1 )
                    {
                        std::cout << "[FIND_PATH] Warning! Edge between nodes " << *it_1 << " and " << *it_2 << " does not exist!" << std::endl;
                    }
                    else
                    {
                        Q.push(CostInt(0,edge));
                        es[edge] = 0;
                    }

                }
            }
            ns_[*it_1] = 0;
        }
    }

    if ( Q.size() < 1 )
    {
        std::cout << "\033[31m" << "[PathFinder] Not enough valid input points!" << "\033[0m" << std::endl;
    }


    /* The path is to contain the series of nodes to get from the source nodes to
     * the target node. To construct this path, the prevs vector is maintained,
     * holding for every visited node the previous node and -1 for ever non-
     * visited node.
     */
    while(!Q.empty())
    {
        // Take the cheapest edge (cost is sum of node costs so far) from the queue
        int u = Q.top().second; // current edge, cheapest pair of nodes so far
        Q.pop();

        // If pair of nodes already visited, continue
        if ( es[u] == -1 )
            continue;

        Graph::const_edge2_iterator edge_it = graph_->beginEdges() + u;
        if ( edge_it->deleted )
        {
            std::cout << "[FIND_PATH] Warning! Skipping deleted edge" << std::endl;
            continue;
        }

        // When the target is reached in current edge's A or B node, trace back path
        if ( target_node != -1 && (edge_it->A == target_node || edge_it->B == target_node))
        {
            tracePath(target_node, path);

            // When finished, return cost to target node
            return ns_[target_node];
        }

        std::vector<int> common_triplets = (graph_->begin() + edge_it->A)->tripletsByPeer(edge_it->B);

        if ( common_triplets.size() == 0 )
            std::cout << "Did not find any common triplets" << std::endl;

        // Run through common triplets of the current pair of nodes
        for ( std::vector<int>::iterator t_it = common_triplets.begin(); t_it != common_triplets.end(); ++t_it )
        {
            // Retrieve the right node from the triplet.
            Graph::const_edge3_iterator trip_it = graph_->beginTriplets() + *t_it;
            if ( trip_it->deleted )
            {
                std::cout << "[FIND_PATH] Warning! Skipping deleted triplet" << std::endl;
                continue;
            }

            int v = trip_it->getThirdNode(edge_it->A,edge_it->B);

            Graph::const_iterator node_it = graph_->begin() + v;
            if ( node_it->deleted )
            {
                std::cout << "[FIND_PATH] Warning! Skipping deleted node" << std::endl;
                continue;
            }

            // TODO: Better weight calculation
            // Check triangle inequality!
            double w;
            double l1 = edge_it->l;
            double l2 = (graph_->beginEdges() + node_it->edgeByPeer(edge_it->A))->l;
            double l3 = (graph_->beginEdges() + node_it->edgeByPeer(edge_it->B))->l;

            double p  = ( l1 + l2 + l3 )/2.0;
            if ( (p-l1)*(p-l2)*(p-l3) < 0 )
            {
                w = 1e38;
            }
            else
            {
                w = weighting3(l1,l2,l3);
            }

            // If path to third node is cheaper than before, update cost to that node, add the cheapest connecting edge to priority queue
            // of potential nodes to visit and record what the previous node was.
            double new_cost = ns_[edge_it->A] + ns_[edge_it->B] + w; // TODO: Now taking sum of node costs plus new cost, is this what I want?
            if (ns_[v] > new_cost)
            {
                ns_[v] = new_cost;

                // Loop through all neighbors of current node (v) and add connecting edges to queue if neighbor is visited
                for ( std::vector<int>::const_iterator e_it = node_it->edges.begin(); e_it !=node_it->edges.end(); ++e_it )
                {
//                    int neighbor = edges[*e_it].getOtherNode(v);
                    int neighbor = (graph_->beginEdges() + *e_it)->getOtherNode(v);

                    // if neighbor is not visited yet, add it to queue
                    if ( ns_[neighbor] < 1e38 )
                        Q.push(CostInt(new_cost, *e_it));
                }

                // Store edge that lead to this node
                prevs_[v] = u;
            }
        }

        // After visiting edge, mark it visited using vector of edge weights
        es[u] = -1;
    }

    // If there is no target node (target_node == -1), the program will get here after calculating paths to ever node in the graph.
    // Now push all nodes in the graph into the path.
    tracePath(target_node,path);
    all_done_ = true;

//    std::cout << "Prevs_:" << std::endl;
//    for ( unsigned int i = 0; i < prevs_.size(); ++i )
//    {
//        std::cout << prevs_[i] << std::endl;
//    }

//    std::cout << std::endl << "ns_:" << std::endl;
//    for ( unsigned int i = 0; i < ns_.size(); ++i )
//    {
//        std::cout << ns_[i] << std::endl;
//    }

    return 0;
}

// -----------------------------------------------------------------------------------------------

void PathFinder::tracePath(const int target_node, Path& path)
{
    std::priority_queue<CostInt, std::vector<CostInt>, std::less<CostInt> > trace;

//    std::vector<Edge2> edges = graph_->getEdge2s();

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
