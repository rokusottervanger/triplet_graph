#ifndef TRIPLET_GRAPH_MEASUREMENT_VISUALIZER_H_
#define TRIPLET_GRAPH_MEASUREMENT_VISUALIZER_H_

#include <vector>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>

#include <tue/config/configuration.h>

#include "triplet_graph/graph_types.h"

namespace triplet_graph
{

class Visualizer
{
    visualization_msgs::Marker points_;
    visualization_msgs::Marker lines_;
    visualization_msgs::MarkerArray msg_;
    ros::Publisher marker_pub_;
    ros::NodeHandle nh_;
    bool is_configured_;

    std::string points_name_, lines_name_;

    void addPointsToMsg(const Measurement&);
    void addAssociatedPointsToMsg(const AssociatedMeasurement&);
    void addLinesToMsg(const Measurement&);

public:
    Visualizer();
    void configure(tue::Configuration &config);
    void publish(const Measurement &measurement);
    void publish(const AssociatedMeasurement& measurement);
    bool isConfigured(){return is_configured_;}
};

} // end namespace triplet_graph

#endif
