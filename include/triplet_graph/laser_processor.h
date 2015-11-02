#ifndef TRIPLET_GRAPH_LASER_PLUGIN_H_
#define TRIPLET_GRAPH_LASER_PLUGIN_H_

namespace triplet_graph
{

class LaserPlugin
{
    void initialize(ed::InitData& init);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void LaserPlugin::process(const triplet_graph::Graph& graph, ed::UpdateRequest& req);
};

}

#endif
