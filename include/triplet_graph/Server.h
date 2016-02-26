#ifndef TRIPLET_GRAPH_ASSOCIATOR_H_
#define TRIPLET_GRAPH_ASSOCIATOR_H_

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include "triplet_graph/graph_types.h"
#include "triplet_graph/Nodes.h"

namespace triplet_graph
{

class Server
{
    Graph* graph_ptr_;
    ros::NodeHandle n_;
    ros::ServiceServer deletion_service_;
    ros::ServiceServer merger_service_;
    ros::ServiceServer rigid_edges_server_;

    bool deleteCallback(Nodes::Request&, Nodes::Response&);
    bool mergeCallback(Nodes::Request&, Nodes::Response&);
    bool rigidEdgesCallback(Nodes::Request&, Nodes::Response&);

public:
    Server(Graph* graph);
};

}

#endif
