#include "triplet_graph/Associator.h"

//#include "triplet_graph/Graph.h"
#include "triplet_graph/Measurement.h"

namespace triplet_graph
{

Associator::Associator():
    associated_(false)
{}

bool Associator::configure(tue::Configuration &config)
{
    return true;
}

// -----------------------------------------------------------------------------------------------

void Associator::setAssociations(const AssociatedMeasurement& associations)
{
    associations_ = associations;
    associated_ = false;
}

// -----------------------------------------------------------------------------------------------

bool Associator::associate(const Graph& graph, const Measurement& measurement)
{
    // Base case
    if ( true )
    {
        associated_ = true;
        return true;
    }

    // Recursive case
    std::vector<geo::Vec3d> prediction;
    nearestNeighbor(measurement, prediction);
    return(associate(graph, measurement)); // Associate function must reduce measurement to the unassociated points, or something...

    // If this takes too long,
    return false;
}

// -----------------------------------------------------------------------------------------------

void Associator::nearestNeighbor( const Measurement& measurement, const std::vector<geo::Vec3d> prediction )
{
    for ( std::vector<geo::Vec3d>::const_iterator it_m = measurement.points.begin(); it_m != measurement.points.end(); ++it_m )
    {
        double best_dist_sq = 1.0e9;
        int best_guess = -1;
        geo::Vec3d best_pos;
        double max_dist_sq = max_association_dist_ * max_association_dist_;

        // Go through nodes in path (nodes to be associated from far to close) to check if one associates with the measured point
        for ( Path::iterator it_p = path_.begin(); it_p != path_.end(); ++it_p )
        {
            int i = *it_p;
            double dx_sq = (*it_m - prediction[i]).length2();
            if ( dx_sq < max_dist_sq )
            {
                if ( dx_sq < best_dist_sq )
                {
                    best_dist_sq = dx_sq;
                    best_pos = *it_m;
                    best_guess = i;
                }
            }
        }

        // Check if an association is made, and if so, push it into associations
        if ( best_guess > -1 )
        {
            associations_.nodes.push_back(best_guess);
            associations_.measurement.points.push_back(best_pos);
        }
        else
        {
            unassociated_points_.points.push_back(*it_m);
        }
    }
}

// -----------------------------------------------------------------------------------------------

bool Associator::getUnassociatedPoints( Measurement& unassociated_points )
{
    if ( associated_ )
    {
        unassociated_points = unassociated_points_;
        return true;
    }
    else
        return false;
}

// -----------------------------------------------------------------------------------------------

bool Associator::getPath(Path& path)
{
    if ( associated_ )
    {
        path = path_;
        return true;
    }
    else
        return false;
}

}
