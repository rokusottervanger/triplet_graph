#include <stdio.h>
#include <iostream>
#include <vector>
#include <queue>
//#include <limits>
//#include <boost/bind.hpp>

#include "triplet_graph/graph_operations.h"
#include "triplet_graph/Graph.h"
#include "triplet_graph/Path.h"

namespace triplet_graph
{

// -----------------------------------------------------------------------------------------------

int findNodeByID(const Graph g, const std::string &id)
{
    for ( Graph::const_iterator it = g.begin(); it != g.end(); it++ )
    {
        if ((*it).id == id)
            return (it - g.begin());
    }
}

// -----------------------------------------------------------------------------------------------

int getConnectingEdge2(const Graph graph, const int Node1, const int Node2)
{
    std::vector<Node> nodes = graph.getNodes();
    std::vector<Edge2> edges = graph.getEdge2s();

    Node n1 = nodes[Node1];
    Node n2 = nodes[Node2];

    // TODO: Make this more efficient by giving nodes a map from node indices to edge indices?
    for( std::vector<int>::iterator it = n1.edges.begin(); it != n1.edges.end(); it++ )
    {
        Edge2 edge = edges[*it];
        if ( edge.A == Node2 || edge.B == Node2 )
            return *it;
    }

    return -1;
}

// -----------------------------------------------------------------------------------------------

std::vector<int> getCommonTriplets(const Graph graph, const int Node1, const int Node2)
{
    std::vector<Node> nodes = graph.getNodes();
    std::vector<Edge3> triplets = graph.getEdge3s();

    Node n1 = nodes[Node1];
    Node n2 = nodes[Node2];

    // TODO: Make this more efficient by giving nodes a set of triplets?
    std::vector<int> common_triplets;

    for( std::vector<int>::iterator it = n1.triplets.begin(); it != n1.triplets.end(); it++ )
    {
        Edge3 triplet = triplets[*it];
        if ( triplet.A == Node2 || triplet.B == Node2 || triplet.C == Node2 )
            common_triplets.push_back(*it);
    }

    return common_triplets;
}

// -----------------------------------------------------------------------------------------------

int getThirdNode(const Edge3& triplet, const int node1, const int node2)
{
    if ( triplet.A == node1 && triplet.B == node2 )
        return triplet.C;
    else if ( triplet.B == node1 && triplet.C == node2 )
        return triplet.A;
    else if ( triplet.C == node1 && triplet.A == node2 )
        return triplet.B;
    else if ( triplet.A == node2 && triplet.B == node1 )
        return triplet.C;
    else if ( triplet.B == node2 && triplet.C == node1 )
        return triplet.A;
    else if ( triplet.C == node2 && triplet.A == node1 )
        return triplet.B;
}

// -----------------------------------------------------------------------------------------------

// TODO: Find a node by its location w.r.t. other nodes?

// -----------------------------------------------------------------------------------------------

Path findPath(const Graph graph, const int source_node1, const int source_node2, const int target_node)
{
    typedef std::pair< double, int > CostEdge; // First is the sum of the costs (so far) to get to the nodes connected by an edge, second is the respective edge index
    const double inf = 1e38;

    // get a copy of the nodes, edges and triplets in the graph
    std::vector<Node>  nodes    = graph.getNodes();
    std::vector<Edge2> edges    = graph.getEdge2s();
    std::vector<Edge3> triplets = graph.getEdge3s();

    int source_edge = getConnectingEdge2(graph,source_node1,source_node2);

    /* The priority queue holds couples of nodes (edges) to be handled, sorted by
     * the sum of the cost to get to those nodes. The cost to get to the first
     * pair of nodes is obviously zero.
     */
    std::priority_queue<CostEdge, std::vector<CostEdge>, std::greater<CostEdge> > Q;
    Q.push(CostEdge(0,source_edge));


    std::vector<double> ns(nodes.size(),inf);
    std::vector<double> es(edges.size(),inf);
    std::vector<double> ts(triplets.size(),inf);
    es[source_edge] = 0;
    ns[source_node1] = 0;
    ns[source_node2] = 0;

    /* The path is to contain the series of nodes to get from the source nodes to
     * the target node. To construct this path, the prevs vector is maintained,
     * holding for every visited node the previous node. The previous
     */
    Path path;      // Path of nodes from source to target;
    std::vector<int> prevs(nodes.size(),-1);
    int prev = source_node1;

    while(!Q.empty())
    {
        // Take the cheapest edge (cost is sum of node costs so far) from the queue
        int u = Q.top().second; // current edge, cheapest pair of nodes so far
        Q.pop();

        // If pair of nodes already visited, continue
        if ( es[u] == -1 )
            continue;

        // When the target is reached, trace back path and return
        if ( edges[u].A == target_node )
        {
            int n = edges[u].A;
            path.push(n);
            while ( n != source_node1 || n!= source_node2 ) // Klopt dit?
            {
                n = prevs[n];
                path.push(n);
            }
            return path;
        }
        else if ( edges[u].B == target_node )
        {
            int n = edges[u].B;
            path.push(n);
            while ( n != source_node1 || n!= source_node2 ) // Klopt dit?
            {
                n = prevs[n];
                path.push(n);
            }
            return path;
        }

        std::vector<int> common_triplets = getCommonTriplets(graph,edges[u].A,edges[u].B);

        // Run through common triplets of the current pair of nodes
        for ( std::vector<int>::iterator t_it = common_triplets.begin(); t_it != common_triplets.end(); t_it++ )
        {
            // Retrieve the right node from the triplet.
            int v = getThirdNode(triplets[*t_it],edges[u].A,edges[u].B);

            // If this triplet was already visited, continue
            if ( ts[*t_it] == -1 )
                continue;
            ts[*t_it] = -1;// TODO: Is this the right place to mark triplet visited?

            // TODO: Calculate weight using the two edges connecting the third node to the two base nodes
            double w = 1.0;

            // If path to third node is cheaper than before, update cost to that node, add the cheapest connecting edge to priority queue
            // of potential nodes to visit and record what the previous node was.
            double new_cost = ns[edges[u].A] + ns[edges[u].B] + w; // TODO: Now taking sum of node costs plus new cost, is this what I want?
            if (ns[v] > new_cost)
            {
                ns[v] = new_cost;
                Q.push(CostEdge(new_cost, getConnectingEdge2(graph,v,edges[u].A)));
                Q.push(CostEdge(new_cost, getConnectingEdge2(graph,v,edges[u].B)));

                // The most expensive node of the current set of nodes must be the previous one, so pich that as the previous
                if ( ns[edges[u].A] > ns[edges[u].B] )
                    prevs[v] = ns[edges[u].A];
                else if ( ns[edges[u].A] <= ns[edges[u].B] )
                    prevs[v] = ns[edges[u].B];
            }
        }

        // After visiting edge, mark it visited using vector of edge weights
        es[u] = -1;
    }
}

}
