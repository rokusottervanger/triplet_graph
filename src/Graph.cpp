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
    // If a triplet exists between the three nodes, it is at least stored in the node that is part of the least triplets, so get node with the least triplets
    int n1, n2, n3;

    int size1 = nodes_[node1].triplets.size();
    int size2 = nodes_[node2].triplets.size();
    int size3 = nodes_[node3].triplets.size();

    if ( size2 < size1 )
    {
        size1 = size2;
        n1 = node2;
        n2 = node1;
        n3 = node3;
    }
    else
    {
        n1 = node1;
        n2 = node2;
        n3 = node3;
    }

    if ( size3 < size1 )
    {
        size1 = size3;
        n1 = node3;
        n2 = node2;
        n3 = node1;
    }

    // Now loop through that node's list of triplets and check if there is one that also refers to the other two nodes
    for ( std::vector<int>::const_iterator it = nodes_[n1].triplets.begin(); it != nodes_[n1].triplets.end(); it++ )
    {
        std::vector<int> v_indices;
        v_indices.push_back(triplets_[*it].A);
        v_indices.push_back(triplets_[*it].B);
        v_indices.push_back(triplets_[*it].C);

        do
        {
            if ( v_indices[0] == node1 && v_indices[1] == node2 && v_indices[2] == node3 )
            {
                std::cout << "Found the same triplet as the one we're trying to add! Only updating order!" << std::endl;
                triplets_[*it] = trip;
            }
        } while  ( std::next_permutation(v_indices.begin(),v_indices.end()) );
    }

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
//    [i].remove();
    // TODO: make node clean itself and its edges up
    // TODO: after running deleteNode, the node is still in the vector
    deleted_nodes_.push_back(i);
    std::cout << "Deleted node " << i << std::endl;
}

// -----------------------------------------------------------------------------------------------

void Graph::update(const Measurements& measurements)
{
    std::cout << "[GRAPH] Updating graph (does nothing right now)" << std::endl;
}

}
