#include "triplet_graph/CornerDetector.h"

#include "triplet_graph/Measurement.h"

#include <string>
#include <ros/node_handle.h>
#include <geolib/sensors/LaserRangeFinder.h>

namespace triplet_graph
{

bool CornerDetector::configure(tue::Configuration &config)
{
    std::string laser_topic;

    config.value("laser_topic", laser_topic);
    config.value("corner_threshold", corner_threshold_);
    config.value("step_size", step_size_);
    config.value("jump_size", jump_size_);

    if ( config.readGroup("visualization"))
    {
        visualizer_.configure(config);
        config.endGroup();
    }
    else
        std::cout << "[CornerDetector] Configure: No visualization parameters found" << std::endl;


    if (config.hasError())
    {
        std::cout << "[CornerDetector] Configure: Config has error: " << config.error() << std::endl;
        return false;
    }

    ros::NodeHandle nh;

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, &CornerDetector::scanCallback, this);

    return true;
}

// -----------------------------------------------------------------------------------------------

void CornerDetector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_msg_ = msg;
}

// -----------------------------------------------------------------------------------------------

void CornerDetector::process(triplet_graph::Measurement& measurement)
{
    if ( !scan_msg_ )
        return;

    measurement.time_stamp = scan_msg_->header.stamp;
    measurement.frame_id = scan_msg_->header.frame_id;

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

    int step_size = 0;
    double step_dist_sq_ = 0.25*0.25; // step size in meters

    std::set<int> added_point_indices;
    for (int i = 0; i < num_beams-step_size; i++ )
    {
        geo::Vec3d A, B, db, dc, N, c;

        for (int j = 0; j < num_beams - i; j++ )
        {
            step_size = j;

            // Get vectors to range points
            A = lrf_model_.rangeToPoint(sensor_ranges[i],i);
            B = lrf_model_.rangeToPoint(sensor_ranges[i+step_size],i+step_size);

            // Calculate vector between point A and point B
            geo::Vec3d db_tmp = B - A;

            if ( db_tmp.length2() < step_dist_sq_ )
                db = db_tmp;
            else
                break;
        }

        // Calculate normal unit vector to db
        N = geo::Mat3d(0,-1,0,1,0,0,0,0,1) * db/db.length();

        double d_max = corner_threshold_;
        geo::Vec3d c_corner;
        int corner_index;

        // For every intermediate point, check if distance to line segment ab is small enough. If too big, it's a corner!
        for (int j = 1; j <= step_size; j++ )
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
            double d = fabs(N.dot(dc));
            if ( d > corner_threshold_ )
            {
                if ( d > d_max )
                {
                    d_max = d;
                    c_corner = c;
                    corner_index = i+j;
                }
            }
        }

        if ( d_max > corner_threshold_ && added_point_indices.find(corner_index) == added_point_indices.end() )
        {
            measurement.points.push_back(c_corner);

            measurement.line_list.push_back(A);
            measurement.line_list.push_back(B);

            added_point_indices.insert(corner_index);
            i = corner_index;
        }
    }

    scan_msg_.reset();

    visualizer_.publish(measurement);
}

}
