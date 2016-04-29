#ifndef TRIPLET_GRAPH_ODOM_TRACKER_H_
#define TRIPLET_GRAPH_ODOM_TRACKER_H_

#include <ros/subscriber.h>
#include <tf/transform_listener.h>

#include <tue/config/configuration.h>
#include <geolib/datatypes.h>

namespace triplet_graph
{

class OdomTracker
{
public:
    OdomTracker();
    ~OdomTracker();
    bool configure(tue::Configuration &config);
    void getDelta(geo::Transform &movement, const ros::Time &time);

private:

    bool have_previous_pose_;
    geo::Pose3D previous_pose_;

    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string sensor_frame_id_;

    tf::TransformListener* tf_listener_;

};

}

#endif
