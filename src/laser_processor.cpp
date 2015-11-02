#include "triplet_graph/laser_processor.h"

#include <string>
#include <ros/node_handle.h>

namespace triplet_graph
{

void LaserPlugin::initialize(ed::InitData& init)
{
    std::string laser_topic;
    init.config.value("laser_topic", laser_topic);

    if (init.config.hasError())
        return;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 10, &LaserPlugin::scanCallback, this);

    tf_listener_ = new tf::TransformListener;

    min_segment_size_pixels_ = 10;
    world_association_distance_ = 0.2;
    segment_depth_threshold_ = 0.2;

    min_cluster_size_ = 0.2;
    max_cluster_size_ = 1.0;

    max_gap_size_ = 10;
}

void LaserPlugin::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_msg_ = msg;
}

void LaserPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{

}


}
