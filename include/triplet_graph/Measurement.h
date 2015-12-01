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

// -----------------------------------------------------------------------------------------------

struct AssociatedMeasurement
{
    // A measurement containing all associated points
    Measurement measurement;

    // A vector of node indices to which the measurement is associated (index of point in measurement is the same as index of node number in nodes)
    std::vector<int> nodes;
};

Measurement operator*(const geo::Transform& lhs, const Measurement& rhs);
AssociatedMeasurement operator*(const geo::Transform& lhs, const AssociatedMeasurement& rhs);


} // end namespace triplet_graph



#endif
