#include <stdio.h>
#include <iostream>
//#include <vector>
//#include <queue>
//#include <limits>
//#include <boost/bind.hpp>

#include "triplet_graph/Graph.h"

namespace triplet_graph
{

int Graph::addNode(const std::string& id)
{
    Node node(id);

    int i;

    if (deleted_nodes_.empty())
    {
        i = nodes_.size();
        nodes_.push_back(node);
    }
    else
    {
        i = deleted_nodes_.back();
        nodes_[i] = node;
        deleted_nodes_.pop_back();
    }

    std::cout << "[GRAPH] Added node with id: '" << id << "'" << std::endl;

    return i;
}

// -----------------------------------------------------------------------------------------------

int Graph::addEdge2(const int& node1, const int& node2, double& length)
{
    Edge2 edge2(node1, node2, length);

    int i;

    if (deleted_edges_.empty())
    {
        i = edges_.size();
        edges_.push_back(edge2);
    }
    else
    {
        i = deleted_edges_.back();
        edges_[i] = edge2;
        deleted_edges_.pop_back();
    }

    std::cout << "[GRAPH] Added edge2 of length " << length << " between node " << node1 << " and node " << node2 << std::endl;

    return i;
}

// -----------------------------------------------------------------------------------------------

int Graph::addEdge3(const int& node1, const int& node2, const int& node3)
{
    if (node1 == node2 || node2 == node3 || node3 == node1)
    {
        std::cout << "\033[31m" << "[GRAPH] ERROR! You're trying to add an edge between three nodes of which at least two are the same. Returning -1" << "\033[0m" << std::endl;
        return -1;
    }

    Edge3 trip(node1, node2, node3);

    int i;

    // TODO: Check if triplet exists or not

    // Edge does not yet exist, so add to graph's edges list
    if (deleted_edges_.empty())
    {
        i = triplets_.size();
        triplets_.push_back(trip);
    }
    else
    {
        i = deleted_triplets_.back();
        triplets_[i] = trip;
        deleted_triplets_.pop_back();
    }

    nodes_[node1].addTriplet(i);
    nodes_[node2].addTriplet(i);
    nodes_[node3].addTriplet(i);

    std::cout << "[GRAPH] Added Edge3 between nodes " << node1 << ", " << node2 << " and " << node3 << std::endl;

    return i;
}

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

void Graph::deleteNode(const int& i)
{
//    nodes_[i].remove();
    // TODO: make node clean itself and its edges up
    deleted_nodes_.push_back(i);
    std::cout << "Deleted node " << i << std::endl;
}

// -----------------------------------------------------------------------------------------------

void Graph::deleteNode(const std::string &id)
{
    int i = findNodeByID(id);
    deleteNode(i);
}

// -----------------------------------------------------------------------------------------------

void Graph::update(const Measurements& measurements)
{
    std::cout << "[GRAPH] Updating graph (does nothing right now)" << std::endl;
}

// -----------------------------------------------------------------------------------------------

int Graph::findNodeByID(const std::string &id)
{
    // TODO: if you do this often (probably not), implement using map from ids to indices
    for ( int i = 0; i <= nodes_.size(); i++)
    {
        if (nodes_[i].id == id)
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
