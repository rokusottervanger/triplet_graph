#include "triplet_graph/Visualizer.h"
#include "triplet_graph/Measurement.h"

namespace triplet_graph
{

Visualizer::Visualizer():
    nh("~"),
    points_name(""),
    lines_name(""),
    is_configured(false)
{}

// ----------------------------------------------------------------------------------------------------

void Visualizer::configure(tue::Configuration& config)
{
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    if ( config.readGroup("points") )
    {
        config.value("name", points_name, tue::REQUIRED);

        points.ns = points_name;
        points.pose.orientation.w = 1.0;

        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.03;
        points.scale.y = 0.03;

        float r = 0, g = 0, b = 0;
        if ( config.readGroup("color") )
        {
            if (!config.value("r",r,tue::OPTIONAL) && !config.value("g",g,tue::OPTIONAL) && !config.value("b",b,tue::OPTIONAL))
                r = g = b = 1.0;
            config.endGroup();
        }
        else
            r = g = b = 1.0;

        points.color.r = r;
        points.color.g = g;
        points.color.b = b;
        points.color.a = 1;

        config.endGroup();
        is_configured = true;
    }

    if ( config.readGroup("lines") )
    {
        config.value("name", lines_name, tue::REQUIRED);

        lines.ns = lines_name;
        lines.pose.orientation.w = 1.0;

        lines.id = 0;
        lines.type = visualization_msgs::Marker::LINE_LIST;
        lines.scale.x = 0.002;
        lines.scale.y = 0.002;

        float r = 0, g = 0, b = 0;
        if ( config.readGroup("color") )
        {
            if (!config.value("r",r,tue::OPTIONAL) && !config.value("g",g,tue::OPTIONAL) && !config.value("b",b,tue::OPTIONAL))
                r = g = b = 1.0;
            config.endGroup();
        }
        else
            r = g = b = 1.0;

        points.color.r = r;
        points.color.g = g;
        points.color.b = b;
        points.color.a = 1;

        config.endGroup();
        is_configured = true;
    }
}

// ----------------------------------------------------------------------------------------------------

void Visualizer::publish(Measurement& measurement)
{
    // TODO: If no lines, don't publish lines; if no points, don't publish points.

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Publish points

    points.points.clear();

    points.header.stamp = measurement.time_stamp;
    points.header.frame_id = measurement.frame_id;

    for ( std::vector<geo::Vec3d>::iterator it = measurement.points.begin(); it != measurement.points.end(); it++ )
    {
        geometry_msgs::Point p;
        p.x = it->getX();
        p.y = it->getY();
        p.z = it->getZ();

        points.points.push_back(p);
    }

    marker_pub.publish(points);

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Publish lines

    lines.points.clear();

    lines.header.stamp = measurement.time_stamp;
    lines.header.frame_id = measurement.frame_id;

    for ( std::vector<geo::Vec3d>::iterator it = measurement.line_list.begin(); it != measurement.line_list.end(); it++ )
    {
        geometry_msgs::Point p;
        p.x = it->getX();
        p.y = it->getY();
        p.z = it->getZ();

        lines.points.push_back(p);
    }

    marker_pub.publish(lines);
}

} // end namespace triplet_graph

