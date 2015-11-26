#include "triplet_graph/OdomTracker.h"

#include <string>
#include <ros/node_handle.h>
#include <geolib/ros/tf_conversions.h>

namespace triplet_graph
{

OdomTracker::OdomTracker(): tf_listener_() {}

void OdomTracker::configure(tue::Configuration &config)
{
    if (config.readGroup("odom_tracker", tue::REQUIRED))
    {
        config.value("map_frame", map_frame_id_);
        config.value("odom_frame", odom_frame_id_);
        config.value("base_link_frame", base_link_frame_id_);

        config.endGroup();
    }
    else
    {
        std::cout << "\033[31m" << "[ODOM TRACKER] Configure: No configuration for odom tracker found!" << "\033[0m" << std::endl;
    }

    if (config.hasError())
        return;

    delete tf_listener_;
    tf_listener_ = new tf::TransformListener;

    return;
}

// -----------------------------------------------------------------------------------------------

void OdomTracker::getDelta(geo::Transform& movement, const ros::Time& time)
{
    if (!tf_listener_->waitForTransform(odom_frame_id_, base_link_frame_id_, time, ros::Duration(1.0)))
    {
        ROS_WARN_STREAM("[ODOM TRACKER] Cannot get transform from '" << odom_frame_id_ << "' to '" << base_link_frame_id_ << "'.");
        return;
    }

    geo::Pose3D odom_to_base_link;

    try
    {
        tf::StampedTransform odom_to_base_link_tf;

        tf_listener_->lookupTransform(odom_frame_id_, base_link_frame_id_, time, odom_to_base_link_tf);

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

        if (!have_previous_pose_)
            return;

        odom_to_base_link = previous_pose_;

        movement = geo::Transform::identity();
    }

}

}
