#include "triplet_graph/Visualizer.h"
#include "triplet_graph/Measurement.h"
#include <sstream>

namespace triplet_graph
{

Visualizer::Visualizer():
    nh_("~"),
    is_configured_(false)
{}

// ----------------------------------------------------------------------------------------------------

void Visualizer::configure(tue::Configuration& config)
{
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);

    ros::Duration ttl;
    double lifetime = 0;
    if ( !config.value("lifetime", lifetime, tue::OPTIONAL) )
        std::cout << "[VISUALIZER] configure: no lifetime specified, using infinite lifetime" << std::endl;
    ttl = ros::Duration(lifetime);

    if ( config.readGroup("points") )
    {
        config.value("name", points_name_, tue::REQUIRED);

        points_.ns = points_name_;
        points_.pose.orientation.w = 1.0;

        points_.id = 0;
        points_.type = visualization_msgs::Marker::POINTS;
        points_.scale.x = 0.03;
        points_.scale.y = 0.03;

        float r = 0, g = 0, b = 0;
        if ( config.readGroup("color") )
        {
            config.value("r",r);
            config.value("g",g);
            config.value("b",b);
            config.endGroup();
        }
        else
        {
            r = g = b = 1.0;
        }

        points_.color.r = r;
        points_.color.g = g;
        points_.color.b = b;
        points_.color.a = 1;

        points_.lifetime = ttl;

        config.endGroup();
        is_configured_ = true;
    }

    if ( config.readGroup("lines") )
    {
        config.value("name", lines_name_, tue::REQUIRED);

        lines_.ns = lines_name_;
        lines_.pose.orientation.w = 1.0;

        lines_.id = 0;
        lines_.type = visualization_msgs::Marker::LINE_LIST;
        lines_.scale.x = 0.002;
        lines_.scale.y = 0.002;

        float r = 0, g = 0, b = 0;
        if ( config.readGroup("color") )
        {
            config.value("r",r);
            config.value("g",g);
            config.value("b",b);
            config.endGroup();
        }
        else
        {
            r = g = b = 1.0;
        }

        lines_.color.r = r;
        lines_.color.g = g;
        lines_.color.b = b;
        lines_.color.a = 1;

        lines_.lifetime = ttl;

        config.endGroup();
        is_configured_ = true;
    }

}

// ----------------------------------------------------------------------------------------------------

void Visualizer::publish(const AssociatedMeasurement& measurement)
{
    if ( !is_configured_ )
        return;

    publish(measurement.measurement);

    if ( measurement.nodes.size() )
    {
        int i = 0;
        for ( std::vector<geo::Vec3d>::const_iterator it = measurement.measurement.points.begin(); it != measurement.measurement.points.end(); it++ )
        {
            visualization_msgs::Marker text_marker;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.scale.z = 0.1;

            text_marker.color.r = 1;
            text_marker.color.g = 1;
            text_marker.color.b = 1;
            text_marker.color.a = 1.0;

            geometry_msgs::Point p;
            p.x = it->getX();
            p.y = it->getY();
            p.z = it->getZ();

            text_marker.lifetime = points_.lifetime;

            text_marker.pose.position = p;
            text_marker.header.frame_id = measurement.measurement.frame_id;
            text_marker.header.stamp = measurement.measurement.time_stamp;
            text_marker.ns = "node_numbers";
            text_marker.id = i;

            std::stringstream ss;
            ss << measurement.nodes[i] << " (" << p.x << ", " << p.y << ")";

            text_marker.text = ss.str();

            msg_.markers.push_back(text_marker);

            ++i;
        }

        marker_pub_.publish(msg_);
        msg_.markers.clear();
    }
}

// ----------------------------------------------------------------------------------------------------

void Visualizer::publish(const Measurement& measurement)
{
    if ( !is_configured_ )
        return;

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Publish points

    if ( measurement.points.size() )
    {
        points_.points.clear();

        points_.header.stamp = measurement.time_stamp;
        points_.header.frame_id = measurement.frame_id;

        for ( std::vector<geo::Vec3d>::const_iterator it = measurement.points.begin(); it != measurement.points.end(); it++ )
        {
            geometry_msgs::Point p;
            p.x = it->getX();
            p.y = it->getY();
            p.z = it->getZ();

            points_.points.push_back(p);
        }

        msg_.markers.push_back(points_);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Publish lines

    if ( measurement.line_list.size() )
    {
        lines_.points.clear();

        lines_.header.stamp = measurement.time_stamp;
        lines_.header.frame_id = measurement.frame_id;

        for ( std::vector<geo::Vec3d>::const_iterator it = measurement.line_list.begin(); it != measurement.line_list.end(); it++ )
        {
            geometry_msgs::Point p;
            p.x = it->getX();
            p.y = it->getY();
            p.z = it->getZ();

            lines_.points.push_back(p);
        }

        msg_.markers.push_back(lines_);
    }

    marker_pub_.publish(msg_);
    msg_.markers.clear();
}

} // end namespace triplet_graph

