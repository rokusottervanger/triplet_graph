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
    std::vector<geo::Vec3d> line_list; // todo: remove this?
    std::vector<double> uncertainties;
    std::string frame_id;

    void append( const geo::Vec3d& point, const double uncertainty);
    void erase( const int index );
    void clear();

};

// -----------------------------------------------------------------------------------------------

struct AssociatedMeasurement
{
    // A measurement containing all associated points
    Measurement measurement;

    // A vector of node indices to which the measurement is associated (index of point in measurement is the same as index of node number in nodes)
    std::vector<int> nodes;

    // node_indices maps the node indices in the graph to indices of that node in the measurement and the nodes vector
    std::map<int,int> node_indices;

    void append( const geo::Vec3d& point, const double uncertainty, const int node );
    void erase( const int index );
    void clear();

};

Measurement operator*(const geo::Transform& lhs, const Measurement& rhs);
AssociatedMeasurement operator*(const geo::Transform& lhs, const AssociatedMeasurement& rhs);


} // end namespace triplet_graph



#endif
