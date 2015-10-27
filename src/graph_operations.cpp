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

int findNodeByID(const Graph& g, const std::string &id)
{
    for ( Graph::const_iterator it = g.begin(); it != g.end(); it++ )
    {
        if ((*it).id == id)
            return (it - g.begin());
    }

    // Node not found, return -1!
    return -1;
}

// -----------------------------------------------------------------------------------------------

int getConnectingEdge2(const Graph& graph, const int Node1, const int Node2)
{
    std::vector<Node> nodes = graph.getNodes();
    std::vector<Edge2> edges = graph.getEdge2s();

    Node n1 = nodes[Node1];

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

int getSecondNode(const Edge2& edge, const int node)
{
    if ( edge.A == node )
        return edge.B;
    else if ( edge.B == node )
        return edge.A;
    else
        return -1;
}

// -----------------------------------------------------------------------------------------------

std::vector<int> getCommonTriplets(const Graph& graph, const int Node1, const int Node2)
{
    std::vector<Node> nodes = graph.getNodes();
    std::vector<Edge3> triplets = graph.getEdge3s();

    Node n1 = nodes[Node1];

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
    else
        return -1;
}

// -----------------------------------------------------------------------------------------------

// TODO: Find a node by its location w.r.t. other nodes, for association?

// -----------------------------------------------------------------------------------------------

Path findPath(const Graph& graph, const int source_node1, const int source_node2, const int target_node)
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
            while ( n != -1 )
            {
                path.push(n);
                n = prevs[n];
            }
            return path;
        }
        else if ( edges[u].B == target_node )
        {
            int n = edges[u].B;
            while ( n != -1 )
            {
                path.push(n);
                n = prevs[n];
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

                // Loop through all neighbors of current node (v) and add connecting edges to queue if neighbor is visited
                for ( std::vector<int>::iterator e_it = nodes[v].edges.begin(); e_it !=nodes[v].edges.end(); e_it++ )
                {
                    int neighbor = getSecondNode(edges[*e_it],v);

                    if ( ns[neighbor] < inf )
                        Q.push(CostEdge(new_cost, *e_it));
                }

                // The most expensive node of the current set of nodes must be the previous one, so pich that as the previous.
                // If cost of both nodes is equal, it does not matter which one we pick.
                if ( ns[edges[u].A] > ns[edges[u].B] )
                    prevs[v] = edges[u].A;
                else
                    prevs[v] = edges[u].B;
            }
        }

        // After visiting edge, mark it visited using vector of edge weights
        es[u] = -1;
    }
}

// -----------------------------------------------------------------------------------------------

bool configure(Graph& g, tue::Configuration &config)
{
    if (config.readArray("objects"))
    {
        while (config.nextArrayItem())
        {
            // Check for the 'enabled' field. If it exists and the value is 0, omit this object. This allows
            // the user to easily enable and disable certain objects with one single flag.
            int enabled;
            if (config.value("enabled", enabled, tue::OPTIONAL) && !enabled)
                continue;

            std::string id;
            if (!config.value("id", id))
            {
                std::cout << "\033[31m" << "[GRAPH] ERROR! Node config has no id" << "\033[0m" << std::endl;
                continue;
            }
            else
                g.addNode(id);
        }
        config.endArray();
    }
    else
        return false;

    if (config.readArray("edges"))
    {
        while(config.nextArrayItem())
        {
            std::string id1, id2;
            if (!config.value("n1", id1) || !config.value("n2", id2))
                continue;

            int n1 = findNodeByID(g,id1);
            int n2 = findNodeByID(g,id2);

            if ( n1 == -1 || n2 == -1 )
                std::cout << "\033[31m" << "[GRAPH] WARNING! Could not find nodes corresponding to edge" << "\033[0m" << std::endl;
            else
            {
                double length;
                if (config.value("length", length, tue::REQUIRED))
                {
                    g.addEdge2(n1,n2,length);
                }
                else
                {
                    std::cout << "\033[31m" << "[GRAPH] WARNING Edge length not defined" << "\033[0m" << std::endl;
                    continue;
                }
            }
        }
        config.endArray();
    }
    else
    {
        std::cout << "\033[31m" << "[GRAPH] ERROR! No edges defined in config" << "\033[0m" << std::endl;
        return false;
    }

    if (config.readArray("triplets"))
    {
        while(config.nextArrayItem())
        {
            std::string id1, id2, id3;
            if (!config.value("n1", id1) || !config.value("n2", id2) || !config.value("n3", id3))
                continue;

            int n1 = findNodeByID(g,id1);
            int n2 = findNodeByID(g,id2);
            int n3 = findNodeByID(g,id3);

            if ( n1 == -1 || n2 == -1 || n3 == -1 )
                std::cout << "\033[31m" << "[GRAPH] WARNING! Could not find nodes corresponding to edge" << "\033[0m" << std::endl;
            else
                g.addEdge3(n1,n2,n3);
        }
    }
    else
    {
        std::cout << "\033[31m" << "[GRAPH] ERROR! No triplets defined in config" << "\033[0m" << std::endl;
        return false;
    }

    return true;
}

}
