#include <stdio.h>
#include <iostream>
//#include <vector>
#include <queue>
#include <limits>
#include <boost/bind.hpp>

#include "triplet_graph/TripletGraph.h"

namespace triplet_graph
{

Node* TripletGraph::addNode(std::string id, Node *n_1, Node *n_2, double side13, double side23)
{
    Node node(id);

    Triplet triplet(n_1, n_2, &node, side23, side13);

    nodes_.push_back(node);
    edges_.push_back(triplet);

    return &nodes_.back();
}

// -----------------------------------------------------------------------------------------------

//Node* TripletGraph::addNode(std::string id)
//{
//    Node node(id);
//    nodes_.push_back(node);

//    std::cout << "[GRAPH] Added node with id: '" << id << "'" << std::endl;

//    return &nodes_.back();
//}

// -----------------------------------------------------------------------------------------------

//Node* TripletGraph::addNode(const Node &node)
//{
//    nodes_.push_back(node);

//    std::cout << "[GRAPH] Added node with id: '" << node.id << "'" << std::endl;

//    return &nodes_.back();
//}

// -----------------------------------------------------------------------------------------------

//Triplet* TripletGraph::addEdge(Node* n1, Node* n2, geo::Pose3D &pose)
//{
//    // Create edge object
//    Triplet edge(n1,n2,pose);

//    // Calculate weight
//    edge.w = edge.pose.t.length2(); // todo: better weight calculation

//    std::list<Edge>::iterator edge_it = std::find(edges_.begin(),edges_.end(),edge);

//    // Check if edge already exists (two edges are the same if they define relations between the same two nodes)
//    // Todo: Decide whether or not an edge from 1 to the 2 is the same as from 2 to 1 (but inverse, of course)
//    if ( edge_it == edges_.end()) // Edge does not yet exist
//    {
//        // Add edge to edges vector
//        edges_.push_back(edge);

//        // Add edge to edges vectors of respective nodes. Todo: invert edge before adding to second node?
//        n1->edges.push_back(&edges_.back());
//        n2->edges.push_back(&edges_.back());

//        std::cout << "[GRAPH] Added edge from '" << n1->id << "' to '" << n2->id << "'" << std::endl;

//        return &edges_.back();
//    }
//    else // Edge already exists
//    {
//        // todo: more advanced updating of edges?
//        *edge_it = edge;
//        return &(*edge_it);
//    }
//}

// -----------------------------------------------------------------------------------------------

//Triplet* TripletGraph::addEdge(Node* n1, Node* n2, double weight)
//{
//    // Create edge object
//    Triplet edge(n1,n2,weight);

//    std::list<Edge>::iterator edge_it = std::find(edges_.begin(),edges_.end(),edge);

//    // Check if edge already exists (two edges are the same if they define relations between the same two nodes)
//    // Todo: Decide whether or not an edge from 1 to the 2 is the same as from 2 to 1 (but inverse, of course)
//    if ( edge_it == edges_.end()) // Edge does not yet exist
//    {
//        // Add edge to edges vector
//        edges_.push_back(edge);

//        // Add edge to edges vectors of respective nodes. Todo: invert edge before adding to second node?
//        n1->edges.push_back(&edges_.back());
//        n2->edges.push_back(&edges_.back());

//        std::cout << "[GRAPH] Added edge from '" << n1->id << "' to '" << n2->id << "'" << std::endl;

//        return &edges_.back();
//    }
//    else // Edge already exists
//    {
//        // todo: more advanced updating of edges?
//        *edge_it = edge;
//        return &(*edge_it);
//    }
//}

// -----------------------------------------------------------------------------------------------

//bool TripletGraph::configure(tue::Configuration &config)
//{
//    std::map<std::string,Node*> nodes;

//    if (config.readArray("objects"))
//    {
//        while (config.nextArrayItem())
//        {
//            Node node;

//            // Check for the 'enabled' field. If it exists and the value is 0, omit this object. This allows
//            // the user to easily enable and disable certain objects with one single flag.
//            int enabled;
//            if (config.value("enabled", enabled, tue::OPTIONAL) && !enabled)
//                continue;

//            std::string id;
//            if (!config.value("id", id))
//            {
//                std::cout << "\033[31m" << "[GRAPH] ERROR! Node config has no id" << "\033[0m" << std::endl;
//                continue;
//            }
//            else
//            {
//                node.id = id;
//                nodes[id] = addNode(node);
//            }
//        }
//        std::cout << "1" << std::endl;
//        config.endArray();
//    }

//    if (config.readArray("relations"))
//    {
//        while(config.nextArrayItem())
//        {
//            std::string id1, id2;
//            if (!config.value("n1", id1) || !config.value("n2", id2))
//                continue;

//            std::map<std::string,Node*>::iterator n1_it = nodes.find(id1);
//            std::map<std::string,Node*>::iterator n2_it = nodes.find(id2);

//            if (n1_it != nodes.end())
//            {
//                Node* n1 = n1_it->second;
//                Node* n2 = n2_it->second;

//                geo::Pose3D pose = geo::Pose3D::identity();
//                if (config.readGroup("pose", tue::REQUIRED))
//                {
//                    config.value("x", pose.t.x);
//                    config.value("y", pose.t.y);
//                    config.value("z", pose.t.z);

//                    double roll = 0, pitch = 0, yaw = 0;
//                    config.value("roll", roll, tue::OPTIONAL);
//                    config.value("pitch", pitch, tue::OPTIONAL);
//                    config.value("yaw", yaw, tue::OPTIONAL);
//                    pose.R.setRPY(roll, pitch, yaw);

//                    config.endGroup();
//                }
//                else
//                {
//                    std::cout << "Could not find pose group" << std::endl;
//                    continue;
//                }
//                addEdge(n1,n2,pose);
//            }
//            else
//            {
//                std::cout << "\033[31m" << "[GRAPH] WARNING! Could not find nodes corresponding to edge" << "\033[0m" << std::endl;
//            }

//            std::cout << "[GRAPH] Added edge from: '" << id1 << "' to '" << id2 << "'" << std::endl;
//        }
//    }
//    return true;
//}

// -----------------------------------------------------------------------------------------------

void TripletGraph::update(Measurements measurements)
{
    std::cout << "[GRAPH] Updating graph" << std::endl;
}

// -----------------------------------------------------------------------------------------------

Node* TripletGraph::findNodeByID(std::string id)
{
    std::list<Node>::iterator n_it = std::find_if(nodes_.begin(),nodes_.end(),boost::bind(&Node::id, _1) == id);
    return &(*n_it);
}

// -----------------------------------------------------------------------------------------------

// Todo: Maybe calculate shortest path tree when adding/updating edges/nodes with robot as root.

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
