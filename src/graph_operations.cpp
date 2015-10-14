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
    std::vector<Node> nodes = g.getNodes();
    for ( int i = 0; i <= nodes.size(); i++)
    {
        if (nodes[i].id == id)
            return i;
    }
}

// -----------------------------------------------------------------------------------------------

//Path TripletGraph::Dijkstra(Node* source, Node* target)
//{
//    typedef std::pair< int, Node* > Neighbor;
//    const double inf = std::numeric_limits<double>::infinity();

//    Node* u;
//    Node* v;
//    double c, w;
//    Path path;
//    std::map<Node*, Node*> prev;

//    std::priority_queue<Neighbor, std::vector<Neighbor>, std::greater<Neighbor> > Q;

//    // Initialize cost map with infinity and queue of unvisited nodes
//    std::map<Node*, double> d;
//    for(std::list<Node>::iterator it = nodes_.begin(); it != nodes_.end(); it++)
//    {
//        Node* n_ptr = &(*it);
//        if (n_ptr != source)
//        {
//            d[n_ptr] = inf;
//        }
//    }
//    Q.push(Neighbor(0,source));
//    d[source] = 0;

//    while(!Q.empty())
//    {
//        // Take the cheapest node from the queue
//        u = Q.top().second; // node
//        c = Q.top().first;  // cost so far
//        Q.pop();

//        // If node already visited, continue
//        if ( d.find(u) == d.end() )
//            continue;

//        // When the target is reached, trace back path and return
//        if ( u == target )
//        {
//            Node* n = u;
//            path.push(n);
//            while ( n != source )
//            {
//                n = prev[n];
//                path.push(n);
//            }
//            return path;
//        }

//        // Run through nodes connected to cheapest node so far
//        for ( std::vector<Edge*>::iterator e_it = u->edges.begin(); e_it != u->edges.end(); e_it++ )
//        {
//            // Retrieve the right nodes from the edges.
//            Edge* e_ptr = *e_it;
//            if( e_ptr->n1->id == u->id )
//                v = e_ptr->n2;
//            else if ( e_ptr->n2->id == u->id )
//                v = e_ptr->n1;
//            else
//            {
//                std::cout << "\033[31m" << "[GRAPH] Warning! Edge does not connect two nodes." << "\033[0m" << std::endl;
//                continue;
//            }

//            // If this node was already visited, continue
//            if ( d.find(v) == d.end() )
//                continue;

//            // Get weight from edge
//            w = e_ptr->w;

//            /* If path to second node is cheaper than before,
//            update cost to that node, add it to priority queue
//            of potential nodes to visit and record from which
//            node this cost came.
//            */
//            if (d[v] > d[u] + w)
//            {
//                d[v] = d[u] + w;
//                Q.push(Neighbor(d[v], v));
//                prev[v] = u;
//            }
//        }

//        // After visiting node, remove it from map of nodes with weights.
//        d.erase(u);
//    }
//}

}
