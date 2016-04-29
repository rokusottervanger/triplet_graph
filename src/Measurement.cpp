#include "triplet_graph/Measurement.h"

namespace triplet_graph
{

Measurement operator*(const geo::Transform& lhs, const triplet_graph::Measurement& rhs)
{
    triplet_graph::Measurement result = rhs;
    for ( unsigned int i = 0; i < rhs.points.size(); ++i )
        result.points[i] = lhs * rhs.points[i];
    for ( unsigned int i = 0; i < rhs.line_list.size(); ++i )
        result.line_list[i] = lhs * rhs.line_list[i];
    return result;
}

// -----------------------------------------------------------------------------------------------

AssociatedMeasurement operator*(const geo::Transform& lhs, const triplet_graph::AssociatedMeasurement& rhs)
{
    triplet_graph::AssociatedMeasurement result = rhs;
    result.measurement = lhs * result.measurement;
    return result;
}

// -----------------------------------------------------------------------------------------------

void Measurement::append(const geo::Vec3d& point, const double uncertainty)
{
    points.push_back(point);
    uncertainties.push_back(uncertainty);
}

// -----------------------------------------------------------------------------------------------

void Measurement::erase(const int index)
{
    points.erase(points.begin()+index);
    uncertainties.erase(uncertainties.begin()+index);
}

// -----------------------------------------------------------------------------------------------

void Measurement::clear()
{
    points.clear();
    uncertainties.clear();
    line_list.clear();
    frame_id = "";
}

// -----------------------------------------------------------------------------------------------

void AssociatedMeasurement::append( const geo::Vec3d& point, const double uncertainty, const int node )
{
    measurement.append(point, uncertainty);
    node_indices[node] = nodes.size();
    nodes.push_back(node);
}

// -----------------------------------------------------------------------------------------------

void AssociatedMeasurement::erase( const int index )
{
    measurement.erase(index);
    nodes.erase(nodes.begin()+index);

    for ( std::map<int,int>::iterator it = node_indices.begin(); it != node_indices.end(); ++it )
    {
        if ( it->second > index )
        {
            it->second -= 1;
        }
    }
}

// -----------------------------------------------------------------------------------------------

void AssociatedMeasurement::clear()
{
    measurement.clear();
    nodes.clear();
    node_indices.clear();
}

} // end namespace triplet_graph

