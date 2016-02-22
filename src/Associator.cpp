#include "triplet_graph/Associator.h"

#include "triplet_graph/Measurement.h"

namespace triplet_graph
{

bool Associator::configure(tue::Configuration &config)
{
    max_iterations_ = 1;
    return true;
}

// -----------------------------------------------------------------------------------------------

bool Associator::associate(triplet_graph::AssociatedMeasurement& associations)
{
    int i = 0;
    while ( i < max_iterations_ )
    {
        ++i;
    }
    return false;
}

}
