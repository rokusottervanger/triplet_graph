#include <stdio.h>
#include <iostream>

#include "triplet_graph/Graph.h"
#include "triplet_graph/Edge2.h"
#include "triplet_graph/Edge3.h"
#include "triplet_graph/Node.h"

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

int Graph::addEdge2(const int node1, const int node2, const double& length)
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

    nodes_[node1].edges.push_back(i);
    nodes_[node2].edges.push_back(i);

    std::cout << "[GRAPH] Added edge2 of length " << length << " between node " << node1 << " and node " << node2 << std::endl;

    return i;
}

// -----------------------------------------------------------------------------------------------

int Graph::addEdge3(const int node1, const int node2, const int node3)
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

void Graph::deleteNode(const int i)
{
    Node node = nodes_[i];
    nodes_[i].id = "";

    for( std::vector<int>::reverse_iterator it = node.triplets.rbegin(); it!= node.triplets.rend(); it++)
    {
        deleteEdge3(*it);
    }
    for( std::vector<int>::reverse_iterator it = node.edges.rbegin(); it != node.edges.rend(); it++ )
    {
        deleteEdge2(*it);
    }

    nodes_[i] = Node("");
    deleted_nodes_.push_back(i);
    std::cout << "Deleted node " << i << std::endl;
    std::cout << "\033[31m" << "[GRAPH] WARNING! Deleting nodes is not completely implemented yet!!!" << "\033[0m" << std::endl;
}

// -----------------------------------------------------------------------------------------------

void Graph::deleteEdge2(const int i)
{
    Edge2 edge = edges_[i];

    if ( nodes_[edge.A].id != "" )
    {
        if ( nodes_[edge.B].edges.size() == 1 )
        {
            std::cout << "\033[31m" << "[GRAPH] WARNING! Leaving behind an unconnected node!" << "\033[0m" << std::endl;
        }
    }
    else if ( nodes_[edge.B].id != "" )
    {
        if ( nodes_[edge.A].edges.size() == 1 )
        {
            std::cout << "\033[31m" << "[GRAPH] WARNING! Leaving behind an unconnected node!" << "\033[0m" << std::endl;
        }
    }

    // For both nodes stored in this edge, find the index to this edge (i) in its list of edges and erase it from that list
    nodes_[edge.A].edges.erase(std::find(nodes_[edge.A].edges.begin(),nodes_[edge.A].edges.end(),i));
    nodes_[edge.B].edges.erase(std::find(nodes_[edge.B].edges.begin(),nodes_[edge.B].edges.end(),i));

    edges_[i].deleted = true;
    deleted_edges_.push_back(i);
    std::cout << "\033[31m" << "[GRAPH] WARNING! Deleting edges is not completely implemented yet!!!" << "\033[0m" << std::endl;
}

// -----------------------------------------------------------------------------------------------

void Graph::deleteEdge3(const int i)
{
    Edge3 triplet = triplets_[i];

    // For every node stored in this triplet, find the index to this triplet (i) in its list of triplets and erase it from that list
    nodes_[triplet.A].triplets.erase(std::find(nodes_[triplet.A].triplets.begin(),nodes_[triplet.A].triplets.end(),i));
    nodes_[triplet.B].triplets.erase(std::find(nodes_[triplet.B].triplets.begin(),nodes_[triplet.B].triplets.end(),i));
    nodes_[triplet.C].triplets.erase(std::find(nodes_[triplet.C].triplets.begin(),nodes_[triplet.C].triplets.end(),i));

    triplets_[i].deleted = true;
    deleted_triplets_.push_back(i);
    std::cout << "\033[31m" << "[GRAPH] WARNING! Deleting triplets is not completely implemented yet!!!" << "\033[0m" << std::endl;
}

// -----------------------------------------------------------------------------------------------

void Graph::setEdgeLength(const int i, const double l)
{
    edges_[i].l = l;
}

// -----------------------------------------------------------------------------------------------

//void Graph::update(const Measurements& measurements)
//{
//    std::cout << "[GRAPH] Updating graph (does nothing right now)" << std::endl;
//}

}
