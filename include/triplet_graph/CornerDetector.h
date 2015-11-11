#ifndef TRIPLET_GRAPH_CORNER_DETECTOR_H_
#define TRIPLET_GRAPH_CORNER_DETECTOR_H_

#include <ros/subscriber.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

#include <tue/config/configuration.h>
#include <geolib/sensors/LaserRangeFinder.h>

#include "triplet_graph/graph_types.h"
#include "triplet_graph/Visualizer.h"

namespace triplet_graph
{

class CornerDetector
{
public:
    void configure(tue::Configuration &config);
    void process(Measurement &measurement);

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::Subscriber sub_scan_;
    sensor_msgs::LaserScan::ConstPtr scan_msg_;

    geo::LaserRangeFinder lrf_model_;

    Visualizer visualizer_;

    double jump_size_;
    int step_size_;
    float corner_threshold_;

};

}

#endif
