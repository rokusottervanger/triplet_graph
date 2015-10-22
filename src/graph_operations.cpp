#include <stdio.h>
#include <iostream>
#include <vector>
//#include <queue>
//#include <limits>
//#include <boost/bind.hpp>

#include "triplet_graph/graph_operations.h"
#include "triplet_graph/Graph.h"

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
    std::vector<int> common_triplets = getCommonTriplets(graph,source_node1,source_node2);
//    std::vector<int> common_neighbors = getCommonNeighbors(graph,common_triplets)

    int u;          // Cheapest node so far
    int v;          // A node connected to node u
    int w;          // Edge connecting u and v
    double c, w;    // Cost to get to node u, current edge weight
    Path path;      // Path of nodes from source to target;
    std::vector<int> prev(nodes.size());

    std::priority_queue<CostEdge, std::vector<CostEdge>, std::greater<CostEdge> > Q;

    std::vector<double> d(nodes.size(),inf);
    std::vector<double> e(edges.size(),inf);
    Q.push(CostEdge(0,source_edge));
    e[source_edge] = 0;
    d[source_node1] = 0;
    d[source_node2] = 0;

    while(!Q.empty())
    {
        // Take the cheapest edge (cost is sum of node costs so far) from the queue
        u = Q.top().second; // edge
        c = Q.top().first;  // cost so far
        Q.pop();

        // If pair of nodes already visited, continue
        if ( e[u] == -1 )
            continue;

        // When the target is reached, trace back path and return
        if ( edges[u].A == target_node )
        {
            int n = edges[u].A;
            path.push(n);
            while ( n != source )
            {
                n = prev[n];
                path.push(n);
            }
            return path;
        }
        else if ( edges[u].B == target_node )
        {
            int n = edges[u].B;
            path.push(n);
            while ( n != source )
            {
                n = prev[n];
                path.push(n);
            }
            return path;
        }

        w = getConnectingEdge2(graph,u,n);
        common_triplets = getCommonTriplets(graph,u,n);

        // Run through nodes connected to cheapest edge so far
        for ( std::vector<int>::iterator t_it = common_triplets.begin(); t_it != common_triplets.end(); t_it++ )
        {
            Edge2 edge = edges_[*e_it];
            v = getThirdNode(triplets[*t_it],u,n);
            // Retrieve the right nodes from the edges.
            if( edge.n1 == u )
                v = edge.n2;
            else if ( edge.n2 == u )
                v = edge.n1;
            else
            {
                std::cout << "\033[31m" << "[GRAPH] Warning! Edge does not connect two nodes." << "\033[0m" << std::endl;
                continue;
            }

            // If this node was already visited, continue
            if ( edge[v] == -1 )
                continue;

            // Get weight from edge
            w = edge.w;

            /* If path to second node is cheaper than before,
            update cost to that node, add it to priority queue
            of potential nodes to visit and record from which
            node this cost came.
            */
            double new_cost = d[u] + w;
            if (d[v] > new_cost)
            {
                d[v] = new_cost;
                Q.push(CostEdge(new_cost, v));
                prev[v] = u;
            }
        }

        // After visiting node, remove it from map of nodes with weights.
        d[u] = -1;
    }
}

}
