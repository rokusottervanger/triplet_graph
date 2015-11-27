#include "triplet_graph/Visualizer.h"
#include "triplet_graph/Measurement.h"

namespace triplet_graph
{

Visualizer::Visualizer():
    nh_("~"),
    is_configured_(false)
{}

// ----------------------------------------------------------------------------------------------------

void Visualizer::configure(tue::Configuration& config)
{
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
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
            r = g = b = 1.0;

        points_.color.r = r;
        points_.color.g = g;
        points_.color.b = b;
        points_.color.a = 1;

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
            if (!config.value("r",r,tue::OPTIONAL) && !config.value("g",g,tue::OPTIONAL) && !config.value("b",b,tue::OPTIONAL))
                r = g = b = 1.0;
            config.endGroup();
        }
        else
            r = g = b = 1.0;

        lines_.color.r = r;
        lines_.color.g = g;
        lines_.color.b = b;
        lines_.color.a = 1;

        config.endGroup();
        is_configured_ = true;
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

        marker_pub_.publish(points_);
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

        marker_pub_.publish(lines_);
    }
}

} // end namespace triplet_graph

