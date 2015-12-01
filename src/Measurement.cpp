#include "triplet_graph/Measurement.h"

namespace triplet_graph
{

triplet_graph::Measurement operator*(const geo::Transform& lhs, const triplet_graph::Measurement& rhs)
{
    triplet_graph::Measurement result = rhs;
    for ( unsigned int i = 0; i < rhs.points.size(); ++i )
        result.points[i] = lhs * rhs.points[i];
    for ( unsigned int i = 0; i < rhs.line_list.size(); ++i )
        result.line_list[i] = lhs * rhs.line_list[i];
    return result;
}

// -----------------------------------------------------------------------------------------------

triplet_graph::AssociatedMeasurement operator*(const geo::Transform& lhs, const triplet_graph::AssociatedMeasurement& rhs)
{
    triplet_graph::AssociatedMeasurement result = rhs;
    result.measurement = lhs * result.measurement;
    return result;
}

} // end namespace triplet_graph

