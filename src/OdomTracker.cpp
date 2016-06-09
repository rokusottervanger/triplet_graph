#include "triplet_graph/OdomTracker.h"

#include <string>
#include <ros/node_handle.h>
#include <geolib/ros/tf_conversions.h>

namespace triplet_graph
{

OdomTracker::OdomTracker(): tf_listener_(), have_previous_pose_(false) {}

OdomTracker::~OdomTracker()
{
    delete tf_listener_;
}

bool OdomTracker::configure(tue::Configuration &config)
{
    config.value("odom_frame", odom_frame_id_);
    config.value("sensor_frame", sensor_frame_id_);

    if (config.hasError())
    {
        std::cout << "[ODOM TRACKER] configure: ERROR: " << config.error() << std::endl;
        return false;
    }

    delete tf_listener_;
    tf_listener_ = new tf::TransformListener;

    return true;
}

// -----------------------------------------------------------------------------------------------

void OdomTracker::getDelta(geo::Transform& movement, const ros::Time& time)
{
    if (!tf_listener_->waitForTransform(odom_frame_id_, sensor_frame_id_, time, ros::Duration(1.0)))
    {
        ROS_WARN_STREAM("[ODOM TRACKER] Cannot get transform from '" << odom_frame_id_ << "' to '" << sensor_frame_id_ << "'.");
        movement = geo::Transform::identity();
        return;
    }

    geo::Pose3D odom_to_base_link;

    try
    {
        tf::StampedTransform odom_to_base_link_tf;

        tf_listener_->lookupTransform(odom_frame_id_, sensor_frame_id_, time, odom_to_base_link_tf);

        geo::convert(odom_to_base_link_tf, odom_to_base_link);

        if (have_previous_pose_)
        {
            movement = previous_pose_.inverse() * odom_to_base_link;
        }
        else
        {
            movement = geo::Transform::identity();
        }

        previous_pose_ = odom_to_base_link;
        have_previous_pose_ = true;
    }
    catch (tf::TransformException e)
    {
        std::cout << "[ODOM TRACKER] " << e.what() << std::endl;

        movement = geo::Transform::identity();
    }

}

// -----------------------------------------------------------------------------------------------

void OdomTracker::getLastOdomPose(geo::Transform& odom)
{
    if ( have_previous_pose_ )
        odom = previous_pose_;
    else
        odom = geo::Transform::identity();
}

}
