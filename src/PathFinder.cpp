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
    return std::min(std::min(l_pp,l_pc1),l_pc2)/std::max(std::max(l_pp,l_pc1),l_pc2);
}

// -----------------------------------------------------------------------------------------------

double PathFinder::findPath(const int target_node, Path& path)
{
    // If path was already calculated...
    if ( target_node != -1 && prevs_[target_node] != -1 )
    {
        // directly trace back that path
        tracePath(target_node, path);
        return ns_[target_node];
    }

    // get a copy of the nodes, edges and triplets in the graph
    std::vector<Node>  nodes    = graph_->getNodes();
    std::vector<Edge2> edges    = graph_->getEdge2s();
    std::vector<Edge3> triplets = graph_->getEdge3s();

    // Track visited edges and triplets
    std::vector<double> es(edges.size(),1e38);
    std::vector<double> ts(triplets.size(),1e38);

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
                    int edge = nodes[*it_1].edgeByPeer(*it_2);
                    if ( edge != -1 )
                    {
                        Q.push(CostInt(0,edge));
                        es[edge] = 0;
                    }
                }
            }
            ns_[*it_1] = 0;
        }
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

        // When the target is reached in current edge's A or B node, trace back path
        if ( target_node != -1 && (edges[u].A == target_node || edges[u].B == target_node))
        {
            tracePath(target_node, path);

            // When finished, return cost to target node
            return ns_[target_node];
        }

        std::vector<int> common_triplets = nodes[edges[u].A].tripletsByPeer(edges[u].B);

        // Run through common triplets of the current pair of nodes
        for ( std::vector<int>::iterator t_it = common_triplets.begin(); t_it != common_triplets.end(); ++t_it )
        {
            // If this triplet was already visited, continue
            if ( ts[*t_it] == -1 )
                continue;
            ts[*t_it] = -1; // TODO: Is this OK?

            // Retrieve the right node from the triplet.
            int v = triplets[*t_it].getThirdNode(edges[u].A,edges[u].B);

            // TODO: Better weight calculation
            // Check triangle inequality!
            double w;
            double l1 = edges[u].l;
            double l2 = edges[nodes[v].edgeByPeer(edges[u].A)].l;
            double l3 = edges[nodes[v].edgeByPeer(edges[u].B)].l;
            double p  = ( l1 + l2 + l3 )/2.0;
            if ( (p-l1)*(p-l2)*(p-l3) < 0 )
                w = 1e38;
            else
                w = weighting1(l1,l2,l3);

            // If path to third node is cheaper than before, update cost to that node, add the cheapest connecting edge to priority queue
            // of potential nodes to visit and record what the previous node was.
            double new_cost = ns_[edges[u].A] + ns_[edges[u].B] + w; // TODO: Now taking sum of node costs plus new cost, is this what I want?
            if (ns_[v] > new_cost)
            {
                ns_[v] = new_cost;

                // Loop through all neighbors of current node (v) and add connecting edges to queue if neighbor is visited
                for ( std::vector<int>::iterator e_it = nodes[v].edges.begin(); e_it !=nodes[v].edges.end(); ++e_it )
                {
                    int neighbor = edges[*e_it].getOtherNode(v);

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

    return 0;
}

// -----------------------------------------------------------------------------------------------

void PathFinder::tracePath(const int target_node, Path& path)
{
    std::priority_queue<CostInt, std::vector<CostInt>, std::less<CostInt> > trace;

    std::vector<Edge2> edges = graph_->getEdge2s();

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
            int na = edges[e].A;
            if ( ns_[na] != 0 && !add_all )
                trace.push(CostInt(ns_[na],na));

            int nb = edges[e].B;
            if ( ns_[nb] != 0 && !add_all )
                trace.push(CostInt(ns_[nb],nb));

            path.push_back(n);
            path.parent_tree.push_back(std::make_pair(na,nb));

            es[n] = -1;
        }
    }

    // Finally, add source nodes to path
    path.insert(path.end(), source_nodes_.begin(), source_nodes_.end());
    path.parent_tree.resize(path.size(),std::make_pair(-1,-1));
}

}
