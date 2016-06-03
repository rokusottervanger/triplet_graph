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
    if ( target_node != -1 && prevs_[target_node] != -1 )
    {
        // directly trace back that path
        std::cout << "Path was already calculated, so just tracing it back" << std::endl;
        tracePath(target_node, path);
        return ns_[target_node];
    }

    // Track visited edges and triplets
    std::vector<double> es(graph_->numEdges(),1e38);

    /* The priority queue holds couples of nodes (edges) to be handled, sorted by
     * the sum of the cost to get to those nodes. The cost to get to the first
     * pair of nodes is obviously zero.
     */
    std::priority_queue<CostInt, std::vector<CostInt>, std::greater<CostInt> > Q;

    std::cout << "Here 1" << std::endl;

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
//                    Graph::const_iterator node_it = graph_->begin() + *it_1;
                    Graph::const_iterator node_it = graph_->iteratorAtIndex(*it_1);
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

    std::cout << "Here 2" << std::endl;

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
        std::cout << "Here 3" << std::endl;
        // Take the cheapest edge (cost is sum of node costs so far) from the queue
        int u = Q.top().second; // current edge, cheapest pair of nodes so far
        Q.pop();

        // If pair of nodes already visited, continue
        if ( es[u] == -1 )
            continue;

//        Graph::const_edge2_iterator edge_it = graph_->beginEdges() + u;
        Graph::const_edge2_iterator edge_it = graph_->edgeIteratorAtIndex(u);
        if ( edge_it->deleted )
        {
            std::cout << "[FIND_PATH] Warning! Skipping deleted edge" << std::endl;
            continue;
        }

        std::cout << "Here u: " << u << std::endl;

        // When the target is reached in current edge's A or B node, trace back path
        if ( target_node != -1 && (edge_it->A == target_node || edge_it->B == target_node))
        {
            std::cout << "Tracing path..." << std::endl;
            tracePath(target_node, path);

            // When finished, return cost to target node
            return ns_[target_node];
        }

        std::cout << "Here 5" << std::endl;
//        std::vector<int> common_triplets = (graph_->begin() + edge_it->A)->tripletsByPeer(edge_it->B);
        std::vector<int> common_triplets = graph_->iteratorAtIndex(edge_it->A)->tripletsByPeer(edge_it->B);
        std::cout << "Here size common triplets: " << common_triplets.size() << std::endl;

        if ( common_triplets.size() == 0 )
            std::cout << "Did not find any common triplets" << std::endl;

        // Run through common triplets of the current pair of nodes
        for ( std::vector<int>::iterator t_it = common_triplets.begin(); t_it != common_triplets.end(); ++t_it )
        {

            std::cout << "Here *t_it: " << *t_it << std::endl;
            // Retrieve the right node from the triplet.
//            Graph::const_edge3_iterator trip_it = graph_->beginTriplets() + *t_it;
            Graph::const_edge3_iterator trip_it = graph_->tripletIteratorAtIndex(*t_it);
            if ( trip_it->deleted )
            {
                std::cout << "[FIND_PATH] Warning! Skipping deleted triplet" << std::endl;
                continue;
            }
            std::cout << "Here 8" << std::endl;

            int v = trip_it->getThirdNode(edge_it->A,edge_it->B);

            std::cout << "Here v = " << v << std::endl;

//            Graph::const_iterator node_it = graph_->begin() + v;
            Graph::const_iterator node_it = graph_->iteratorAtIndex(v);

            std::cout << "Here 10" << std::endl;
            if ( node_it->deleted )
            {
                std::cout << "[FIND_PATH] Warning! Skipping deleted node, this should never happen using graph iterators!" << std::endl;
                continue;
            }


            double w;
            // Edge between parents:
            double l3 = edge_it->l;

            std::cout << "Here 11" << std::endl;
//            double l1 = (graph_->beginEdges() + node_it->edgeByPeer(edge_it->A))->l;    // Edge between parent A and current node
            double l1 = graph_->edgeIteratorAtIndex(node_it->edgeByPeer(edge_it->A))->l;    // Edge between parent A and current node

            std::cout << "Here l1: " << l1 << std::endl;
//            double l2 = (graph_->beginEdges() + node_it->edgeByPeer(edge_it->B))->l;    // Edge between parent B and current node
            double l2 = graph_->edgeIteratorAtIndex(node_it->edgeByPeer(edge_it->B))->l;    // Edge between parent B and current node

            std::cout << "Here l2: " << l2 << std::endl;

            // Check triangle inequality!
            double p  = ( l1 + l2 + l3 )/2.0;
            double x;
            if ( (p-l1)*(p-l2)*(p-l3) < 0 )
            {
                w = 1e38;
            }
            else
            {
//                w = weighting(l1,l2,l3);

                double l1_sq = l1*l1;
                double l2_sq = l2*l2;
                double l3_sq = l3*l3;

                x = (l1_sq-l2_sq+l3_sq)/(2*l3);
                double x_sq = x*x;

                double dx_sq = l1_sq/l3_sq + l2_sq/l3_sq + 1/4.0;
                double thing = 2 * l1 - 2*(l1/l3)*x; // TODO naming?
                double dy_sq = 1/(l1_sq-x_sq) * thing*thing + (4*l2_sq*x_sq)/(l3_sq*(l1_sq-x_sq))+x_sq/(l1_sq-x_sq);

                w =  dx_sq+dy_sq;
            }

            // If path to third node is cheaper than before, update cost to that node, add the cheapest connecting edge to priority queue
            // of potential nodes to visit and record what the previous node was.
//            double new_cost = ns_[edge_it->A] + ns_[edge_it->B] + w; // New cost is sum of (squared) parent node costs plus (squared) step cost

            double a = ns_[edge_it->A];
            double a_sq = a*a;
            double b = ns_[edge_it->B];
            double b_sq = b*b;

            double new_cost = a_sq + (a_sq + b_sq) * l1*l1/(l3*l3) + a_sq*(1-2*x/l3) + w;


            if (ns_[v] > new_cost)
            {
                ns_[v] = new_cost;

                // Loop through all neighbors of current node (v) and add connecting edges to queue if neighbor is visited
                for ( std::vector<int>::const_iterator e_it = node_it->edges.begin(); e_it !=node_it->edges.end(); ++e_it )
                {
                    std::cout << "Here 14" << std::endl;
//                    int neighbor = (graph_->beginEdges() + *e_it)->getOtherNode(v);
                    int neighbor = graph_->edgeIteratorAtIndex(*e_it)->getOtherNode(v);
                    std::cout << "Neighbor: " << neighbor << std::endl;

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

    std::cout << "All done, tracing path..." << std::endl;
    tracePath(target_node,path);

    std::cout << "Path traced, returning " << std::endl;
    all_done_ = true;

    std::cout << "Now really returning!!!" << std::endl;

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
//            Graph::const_edge2_iterator edge_it = graph_->beginEdges() + e;
            Graph::const_edge2_iterator edge_it = graph_->edgeIteratorAtIndex(e);
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
