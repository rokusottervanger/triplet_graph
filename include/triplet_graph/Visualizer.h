#ifndef TRIPLET_GRAPH_MEASUREMENT_VISUALIZER_H_
#define TRIPLET_GRAPH_MEASUREMENT_VISUALIZER_H_

#include <vector>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>

#include <tue/config/configuration.h>

#include "triplet_graph/graph_types.h"

namespace triplet_graph
{

class Visualizer
{
    visualization_msgs::Marker points;
    visualization_msgs::Marker lines;
    ros::Publisher marker_pub;
    ros::NodeHandle nh;
    bool is_configured;

    std::string points_name, lines_name;

public:
    Visualizer();
    void configure(tue::Configuration &config);
    void publish(Measurement &measurement);
    bool isConfigured(){return is_configured;}
};

} // end namespace triplet_graph

#endif
