#include "triplet_graph/laser_processor.h"

#include "triplet_graph/Measurement.h"

#include <string>
#include <ros/node_handle.h>
#include <geolib/sensors/LaserRangeFinder.h>
#include <visualization_msgs/Marker.h>


namespace triplet_graph
{

void LaserPlugin::initialize(tue::Configuration &config)
{
    std::string laser_topic;
    config.value("laser_topic", laser_topic);
    config.value("corner_threshold", corner_threshold_);
    config.value("step_size", step_size_);
    config.value("jump_size", jump_size_);

    if (config.hasError())
        return;

    ros::NodeHandle nh;

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, &LaserPlugin::scanCallback, this);

//    tf_listener_ = new tf::TransformListener;
}

void LaserPlugin::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_msg_ = msg;
}

void LaserPlugin::process(triplet_graph::Measurement& measurement)
{
    if ( !scan_msg_ )
    {
        std::cout << "No scan messages received" << std::endl;
        return;
    }

    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    std::vector<float> sensor_ranges(scan_msg_->ranges.size());
    for(unsigned int i = 0; i < scan_msg_->ranges.size(); ++i)
    {
        float r = scan_msg_->ranges[i];
        if (r > scan_msg_->range_max)
            sensor_ranges[i] = r;
        else if (r == r && r > scan_msg_->range_min)
            sensor_ranges[i] = r;
        else
            sensor_ranges[i] = 0;
    }

    unsigned int num_beams = sensor_ranges.size();

    if (lrf_model_.getNumBeams() != num_beams)
    {
        lrf_model_.setNumBeams(num_beams);
        lrf_model_.setAngleLimits(scan_msg_->angle_min, scan_msg_->angle_max);
    }

    // - - - - - - - - - - - - - - - - - -
    // Filter laser data (get rid of ghost points)

    for(unsigned int i = 1; i < num_beams - 1; ++i)
    {
        float rs = sensor_ranges[i];
        // Get rid of points that are isolated from their neighbours
        if (std::abs(rs - sensor_ranges[i - 1]) > 0.1 && std::abs(rs - sensor_ranges[i + 1]) > 0.1)  // TODO: magic number
        {
            sensor_ranges[i] = sensor_ranges[i - 1];
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Find corners

    std::set<int> added_point_indices;
    for (int i = 0; i < num_beams-step_size_; i++ )
    {
        geo::Vec3d A, B, db, dc, N, c;

        // Get vectors to range points
        A = lrf_model_.rangeToPoint(sensor_ranges[i],i);
        B = lrf_model_.rangeToPoint(sensor_ranges[i+step_size_],i+step_size_);

        // Calculate vector between point A and point B
        db = B - A;

        // Calculate normal unit vector to db
        N = geo::Mat3d(0,-1,0,1,0,0,0,0,1) * db/db.length();

        double d_max = corner_threshold_;
        geo::Vec3d c_corner;
        int corner_index;

        // For every intermediate point, check if distance to line segment ab is small enough. If too big, it's a corner!
        for (int j = 1; j <= step_size_; j++ )
        {
            // If a jump occurs, move on to after jump
            if ( fabs(sensor_ranges[i+j] - sensor_ranges[i+j-1]) > jump_size_ )
            {
                d_max = corner_threshold_;
                i = i+j+1;
                break;
            }
            c = lrf_model_.rangeToPoint(sensor_ranges[i+j],i+j);
            dc = c - A;
            double d = N.dot(dc);
            if ( d > corner_threshold_*sensor_ranges[i+j] )
            {
                if ( d > d_max || d < -d_max )
                {
                    d_max = fabs(d);
                    c_corner = c;
                    corner_index = i+j;
                }
            }
        }

        if ( d_max > corner_threshold_ && added_point_indices.find(corner_index) == added_point_indices.end() )
        {
            measurement.points.push_back(c_corner);
            added_point_indices.insert(corner_index);
        }
    }

    scan_msg_.reset();
}

class Visualizer
{
public:
    visualization_msgs::Marker points;
    ros::Publisher marker_pub;
    std::string frame_id;
    ros::NodeHandle nh;

    Visualizer(): nh("~"){}

    void configure(tue::Configuration config)
    {
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        config.value("frame_id", frame_id);

        points.header.frame_id = frame_id;
        points.ns = "laser_processor";
        points.pose.orientation.w = 1.0;

        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.03;
        points.scale.y = 0.03;

        points.color.g = 1.0f;
        points.color.a = 1.0;
    }

    void publish(Measurement measurement)
    {
        points.points.clear();

        for ( std::vector<geo::Vec3d>::iterator it = measurement.points.begin(); it != measurement.points.end(); it++ )
        {
            geometry_msgs::Point p;
            p.x = it->getX();
            p.y = it->getY();
            p.z = it->getZ();

            points.points.push_back(p);
        }

        marker_pub.publish(points);

    }
};


}


int main(int argc, char** argv)
{
    tue::Configuration config;

    ros::init(argc, argv, "laser_processor");

    // Parse arguments
    if ( argc < 2 )
    {
        std::cout << "Usage: \n\n        laser_processor LASER_PROCESSOR_CONFIG.yaml" << std::endl;
        return 1;
    }

    std::string config_filename = argv[1];
    config.loadFromYAMLFile(config_filename);

    if (config.hasError())
    {
        std::cout << std::endl << "Could not load laser processor configuration file:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    triplet_graph::LaserPlugin laserPlugin;

    laserPlugin.initialize(config);

    ros::Rate loop_rate(15);

    triplet_graph::Visualizer vis;
    vis.configure(config);


    while (ros::ok())
    {
        triplet_graph::Measurement measurement;

        laserPlugin.process(measurement);
        vis.publish(measurement);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
