#ifndef TRIPLET_GRAPH_MEASUREMENT_H_
#define TRIPLET_GRAPH_MEASUREMENT_H_

#include <vector>
#include <string>

#include <ros/time.h>

#include <geolib/datatypes.h>

namespace triplet_graph
{

struct Measurement
{
    ros::Time time_stamp;
    std::vector<geo::Vec3d> points;
    std::vector<geo::Vec3d> line_list;
    std::string frame_id;

};



} // end namespace triplet_graph

#endif
