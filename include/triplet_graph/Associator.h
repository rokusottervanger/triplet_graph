#ifndef TRIPLET_GRAPH_ASSOCIATOR_H_
#define TRIPLET_GRAPH_ASSOCIATOR_H_

#include <tue/config/configuration.h>

#include "triplet_graph/graph_types.h"

namespace triplet_graph
{

class Associator
{
public:
    bool configure(tue::Configuration &config);
    bool associate(AssociatedMeasurement &associations);

private:
    int max_iterations_;


};

}

#endif
