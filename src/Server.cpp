#include "triplet_graph/Server.h"

#include "triplet_graph/Graph.h"
#include "triplet_graph/graph_operations.h"
#include <ros/service_server.h>

namespace triplet_graph
{

Server::Server(Graph* graph)
{
    graph_ptr_ = graph;

    deletion_service_ = n_.advertiseService("delete_nodes", &Server::deleteCallback, this);
    merger_service_ = n_.advertiseService("merge_nodes", &Server::mergeCallback, this);
    rigid_edges_server_ = n_.advertiseService("rigidify_edges", &Server::rigidEdgesCallback, this);
}

bool Server::deleteCallback(Nodes::Request& req, Nodes::Response& res)
{
    ROS_WARN_NAMED("Graph Server", "Deleting nodes is not implemented yet");
    return true;
}

bool Server::mergeCallback(Nodes::Request& req, Nodes::Response& res)
{
    ROS_WARN_NAMED("Graph Server", "Merging nodes is not implemented yet");
    return true;
}

bool Server::rigidEdgesCallback(Nodes::Request& req, Nodes::Response& res)
{
    setRigidEdges(*graph_ptr_,req.nodes);
    return true;
}

} // End namespace triplet_graph
