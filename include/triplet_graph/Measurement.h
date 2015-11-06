#ifndef TRIPLET_GRAPH_MEASUREMENT_H_
#define TRIPLET_GRAPH_MEASUREMENT_H_

#include <vector>
#include <geolib/math_types.h>

namespace triplet_graph
{

class Measurement
{

public:

    std::vector<geo::Vec3d> points;
    std::vector<geo::Vec3d> segments;

private:

};

//typedef std::vector<triplet_graph::Measurement> Measurements;

} // end namespace graph_map

#endif
