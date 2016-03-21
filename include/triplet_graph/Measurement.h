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
    std::vector<geo::Vec3d> line_list; // TODO: remove this?
    std::vector<double> uncertainties; // Uncertainties corresponding to points TODO: fill this when calculating positions in graph and using sensor model in corner detector!
    std::string frame_id;

};

// -----------------------------------------------------------------------------------------------
// TODO: inherit from instead of contain measurement?
struct AssociatedMeasurement
{
    // A measurement containing all associated points
    Measurement measurement;

    // A vector of node indices to which the measurement is associated (index of point in measurement is the same as index of node number in nodes)
    std::vector<int> nodes;

    std::map<int,int> node_indices;
};

Measurement operator*(const geo::Transform& lhs, const Measurement& rhs);
AssociatedMeasurement operator*(const geo::Transform& lhs, const AssociatedMeasurement& rhs);


} // end namespace triplet_graph



#endif
