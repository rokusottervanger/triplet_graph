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

//    std::cout << "[GRAPH] Added node with id: '" << id << "'" << std::endl;

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
    nodes_[node1].edge_by_peer_[node2] = i;
    nodes_[node1].peer_by_edge_[i] = node2;

    nodes_[node2].edges.push_back(i);
    nodes_[node2].edge_by_peer_[node1] = i;
    nodes_[node2].peer_by_edge_[i] = node1;

//    std::cout << "\033[31m" << "[GRAPH] Added edge2 of length " << length << " between node " << node1 << " and node " << node2 << "\033[0m" << std::endl;

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

    int edge1 = nodes_[node1].edgeByPeer(node2);
    int edge2 = nodes_[node2].edgeByPeer(node3);
    int edge3 = nodes_[node3].edgeByPeer(node1);

    if ( edge1 == -1 || edge2 == -1 || edge3 == -1 )
    {
        std::cout << "[GRAPH] AddEdge3: One of the edges does not exist, not adding triplet and returning -1" << std::endl;
        return -1;
    }

    Edge3 trip(node1, node2, node3);

    int i;

    i = edges_[edge1].tripletByNode(node3);

    if ( i > -1 )
    {
        // Triplet already exists, so overwrite old triplet and return
        triplets_[i] = trip;
        return i;
    }

    // triplet does not yet exist, so add to graph's triplets list
    if (deleted_triplets_.empty())
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


    // Add triplet to all nodes and edges that are part of this triplet:
    nodes_[node1].addTriplet(i);
    nodes_[node1].triplets_by_peer_[node2].push_back(i);
    nodes_[node1].triplets_by_peer_[node3].push_back(i);
    edges_[nodes_[node1].edgeByPeer(node2)].triplet_by_node_[node3] = i;


    nodes_[node2].addTriplet(i);
    nodes_[node2].triplets_by_peer_[node1].push_back(i);
    nodes_[node2].triplets_by_peer_[node3].push_back(i);
    edges_[nodes_[node2].edgeByPeer(node3)].triplet_by_node_[node1] = i;


    nodes_[node3].addTriplet(i);
    nodes_[node3].triplets_by_peer_[node1].push_back(i);
    nodes_[node3].triplets_by_peer_[node2].push_back(i);
    edges_[nodes_[node3].edgeByPeer(node1)].triplet_by_node_[node2] = i;

//    std::cout << "[GRAPH] Added Edge3 between nodes " << node1 << ", " << node2 << " and " << node3 << std::endl;

    return i;
}

// -----------------------------------------------------------------------------------------------

void Graph::deleteNode(const int i)
{
    Node node = nodes_[i];

    // Dissolve any connected triplets
    for( std::vector<int>::reverse_iterator it = node.triplets.rbegin(); it!= node.triplets.rend(); it++)
    {
        deleteEdge3(*it);
    }
    // Dissolve any connected edges
    for( std::vector<int>::reverse_iterator it = node.edges.rbegin(); it != node.edges.rend(); it++ )
    {
        deleteEdge2(*it);
    }

    nodes_[i] = Node("");
    deleted_nodes_.push_back(i);
    std::cout << "Deleted node " << i << std::endl;
    std::cout << "\033[31m" << "[GRAPH] WARNING! Handling deleted nodes is not completely implemented yet!" << "\033[0m" << std::endl;
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

    // For both nodes stored in this edge, remove map item from peer to this edge
    std::map<int,int>::iterator it = nodes_[edge.A].edge_by_peer_.find(edge.B);
    nodes_[edge.A].edge_by_peer_.erase(it);

    it = nodes_[edge.B].edge_by_peer_.find(edge.A);
    nodes_[edge.B].edge_by_peer_.erase(it);

    it = nodes_[edge.A].peer_by_edge_.find(i);
    nodes_[edge.A].peer_by_edge_.erase(it);

    it = nodes_[edge.B].peer_by_edge_.find(i);
    nodes_[edge.B].peer_by_edge_.erase(it);

    edges_[i].deleted = true;
    deleted_edges_.push_back(i);
    std::cout << "\033[31m" << "[GRAPH] WARNING! Handling deleted edges is not completely implemented yet!" << "\033[0m" << std::endl;
    // TODO: make sure that edge cannot be found in graph anymore after deletion (using edge iterator?)
}

// -----------------------------------------------------------------------------------------------

void Graph::deleteEdge3(const int i)
{
    Edge3 triplet = triplets_[i];

    // For every node stored in this triplet, find the index to this triplet (i) in its list of triplets and erase it from that list
    nodes_[triplet.A].triplets.erase(std::find(nodes_[triplet.A].triplets.begin(),nodes_[triplet.A].triplets.end(),i));
    nodes_[triplet.B].triplets.erase(std::find(nodes_[triplet.B].triplets.begin(),nodes_[triplet.B].triplets.end(),i));
    nodes_[triplet.C].triplets.erase(std::find(nodes_[triplet.C].triplets.begin(),nodes_[triplet.C].triplets.end(),i));

    // For every edge part of this triplet, remove the triplet from its map from (third) nodes to triplets
    {
        int e1 = nodes_[triplet.A].edgeByPeer(triplet.B);
        int e2 = nodes_[triplet.B].edgeByPeer(triplet.C);
        int e3 = nodes_[triplet.C].edgeByPeer(triplet.A);

        std::map<int,int>::iterator it;
        it = edges_[e1].triplet_by_node_.find(triplet.C);
        edges_[e1].triplet_by_node_.erase(it);
        it = edges_[e2].triplet_by_node_.find(triplet.A);
        edges_[e2].triplet_by_node_.erase(it);
        it = edges_[e3].triplet_by_node_.find(triplet.B);
        edges_[e3].triplet_by_node_.erase(it);
    }

    // For every node part of this triplet, remove the triplet from its map from (second) nodes to shared triplets
    {
        std::vector<int>::iterator it;
        std::vector<int> vector;

        // node A
        vector = nodes_[triplet.A].triplets_by_peer_[triplet.B];
        it = std::find(vector.begin(),vector.end(),i);
        nodes_[triplet.A].triplets_by_peer_[triplet.B].erase(it);

        vector = nodes_[triplet.A].triplets_by_peer_[triplet.C];
        it = std::find(vector.begin(),vector.end(),i);
        nodes_[triplet.A].triplets_by_peer_[triplet.C].erase(it);

        // node B
        vector = nodes_[triplet.B].triplets_by_peer_[triplet.A];
        it = std::find(vector.begin(),vector.end(),i);
        nodes_[triplet.B].triplets_by_peer_[triplet.A].erase(it);

        vector = nodes_[triplet.B].triplets_by_peer_[triplet.C];
        it = std::find(vector.begin(),vector.end(),i);
        nodes_[triplet.B].triplets_by_peer_[triplet.C].erase(it);

        // node C
        vector = nodes_[triplet.C].triplets_by_peer_[triplet.A];
        it = std::find(vector.begin(),vector.end(),i);
        nodes_[triplet.C].triplets_by_peer_[triplet.A].erase(it);

        vector = nodes_[triplet.C].triplets_by_peer_[triplet.B];
        it = std::find(vector.begin(),vector.end(),i);
        nodes_[triplet.C].triplets_by_peer_[triplet.B].erase(it);
    }

    triplets_[i].deleted = true;
    deleted_triplets_.push_back(i);
    std::cout << "\033[31m" << "[GRAPH] WARNING! Handling deleted triplets is not completely implemented yet!" << "\033[0m" << std::endl;
    // TODO: make sure that triplet cannot be found in graph anymore after deletion (using triplet iterator?)
    // TODO: Why would you want to do this? And does that mean that you want to remove the involved edges as well?
}

// -----------------------------------------------------------------------------------------------

void Graph::setEdgeLength(const int i, const double l)
{
    edges_[i].l = l;
}

// -----------------------------------------------------------------------------------------------

void Graph::flipTriplet(const int i)
{
    int tmp = triplets_[i].B;
    triplets_[i].B = triplets_[i].C;
    triplets_[i].C = tmp;
}

}
