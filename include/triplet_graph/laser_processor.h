#ifndef TRIPLET_GRAPH_LASER_PLUGIN_H_
#define TRIPLET_GRAPH_LASER_PLUGIN_H_

#include <ros/subscriber.h>
#include <tf/transform_listener.h>
#include <tue/config/configuration.h>
#include <sensor_msgs/LaserScan.h>
#include "triplet_graph/graph_types.h"
#include "geolib/sensors/LaserRangeFinder.h"

namespace triplet_graph
{

class LaserPlugin
{
public:
    void configure(tue::Configuration &config);
    void process(Measurement &measurement);

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::Subscriber sub_scan_;
//    tf::TransformListener* tf_listener_;
    sensor_msgs::LaserScan::ConstPtr scan_msg_;

    geo::LaserRangeFinder lrf_model_;

    double jump_size_;
    int step_size_;
    float corner_threshold_;

};

}

#endif
